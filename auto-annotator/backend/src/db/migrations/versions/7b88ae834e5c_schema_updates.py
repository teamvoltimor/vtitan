"""schema_updates

Revision ID: 7b88ae834e5c
Revises: 885a98bab4ca
Create Date: 2026-05-25 13:10:03.824544

"""
from typing import Sequence, Union

from alembic import op
import sqlalchemy as sa


# revision identifiers, used by Alembic.
revision: str = '7b88ae834e5c'
down_revision: Union[str, Sequence[str], None] = '885a98bab4ca'
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    """Upgrade schema."""
    import importlib.resources
    sql_text = importlib.resources.files("src.db.sql").joinpath("002_schema_updates.sql").read_text()
    
    op.execute(sa.text("PRAGMA foreign_keys=OFF"))
    for statement in sql_text.split(";"):
        stmt = statement.strip()
        if stmt:
            op.execute(sa.text(stmt))
    op.execute(sa.text("PRAGMA foreign_keys=ON"))


def downgrade() -> None:
    """Downgrade schema."""
    pass
