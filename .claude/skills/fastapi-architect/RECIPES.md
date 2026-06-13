# FastAPI Recipes

Reference implementations for the patterns in [SKILL.md](SKILL.md).

## Project structure

```
src/myapp/
├── main.py                  # FastAPI instance, router includes, lifespan
├── config.py                # Settings via pydantic-settings
├── deps.py                  # shared dependencies (DB pool, auth, etc.)
├── errors.py                # RFC 7807 problem-details exception handlers
├── users/
│   ├── __init__.py
│   ├── router.py            # APIRouter, endpoints, response models
│   ├── service.py           # business logic (no HTTP, no SQL)
│   ├── repo.py              # data access (psycopg + queries/*.sql)
│   ├── schemas.py           # Pydantic request / response models
│   └── queries/
│       ├── get_user_by_id.sql
│       └── insert_user.sql
├── orders/
└── tests/
```

## Pydantic schemas — three shapes per resource

```python
class UserCreate(BaseModel):          # request body for POST
    model_config = ConfigDict(extra="forbid")
    email: EmailStr
    password: SecretStr

class UserUpdate(BaseModel):          # request body for PATCH (partial)
    model_config = ConfigDict(extra="forbid")
    email: EmailStr | None = None

class UserResponse(BaseModel):        # response body
    model_config = ConfigDict(from_attributes=True)
    id: UUID
    email: EmailStr
    created_at: datetime
```

## Dependency type aliases

```python
DB = Annotated[AsyncConnection, Depends(get_db)]
CurrentUser = Annotated[User, Depends(get_current_user)]

@router.get("/me")
async def me(user: CurrentUser) -> UserResponse: ...
```

## Lifespan & settings

```python
@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncIterator[None]:
    async with psycopg_pool.AsyncConnectionPool(settings.db_dsn) as pool:
        app.state.db_pool = pool
        async with httpx.AsyncClient(timeout=10) as client:
            app.state.http = client
            yield

app = FastAPI(lifespan=lifespan)

class Settings(BaseSettings):
    model_config = SettingsConfigDict(env_file=".env", env_prefix="MYAPP_")
    db_dsn: PostgresDsn
    jwt_secret: SecretStr
    jwt_alg: Literal["HS256", "RS256"] = "HS256"

@lru_cache
def get_settings() -> Settings: return Settings()
```

## Auth — Pattern A (in-house OAuth2 + JWT)

```python
oauth2 = OAuth2PasswordBearer(tokenUrl="/v1/auth/login")

async def get_current_user(
    token: Annotated[str, Depends(oauth2)],
    settings: Annotated[Settings, Depends(get_settings)],
    db: DB,
) -> User:
    try:
        payload = jwt.decode(
            token,
            settings.jwt_secret.get_secret_value(),
            algorithms=[settings.jwt_alg],
        )
    except jwt.PyJWTError:
        raise HTTPException(401, "Invalid token")
    return await load_user(db, payload["sub"])

CurrentUser = Annotated[User, Depends(get_current_user)]
```

**Pattern B — external IdP**: `pyjwt`'s `PyJWKClient` for JWKS verification; cache the client (`@lru_cache`) so JWKS isn't fetched per request. Verify `aud` and `iss` explicitly.

## Authorization per route

```python
def require_scope(scope: str):
    async def _checker(user: CurrentUser):
        if scope not in user.scopes:
            raise HTTPException(403)
    return _checker

@router.delete("/users/{id}", dependencies=[Depends(require_scope("users:delete"))])
async def delete_user(...): ...
```

## RFC 7807 problem-details handler

```python
class Problem(BaseModel):
    type: str = "about:blank"
    title: str
    status: int
    detail: str | None = None
    instance: str | None = None

@app.exception_handler(DomainError)
async def domain_error_handler(req: Request, exc: DomainError):
    return JSONResponse(
        status_code=exc.status,
        content=Problem(
            type=f"https://errors.myapp.io/{exc.code}",
            title=exc.title, status=exc.status, detail=str(exc),
            instance=str(req.url),
        ).model_dump(),
        media_type="application/problem+json",
    )
```
