"""initial

Revision ID: 885a98bab4ca
Revises: 
Create Date: 2026-05-25 13:09:19.136963

"""
from typing import Sequence, Union

from alembic import op
import sqlalchemy as sa


# revision identifiers, used by Alembic.
revision: str = '885a98bab4ca'
down_revision: Union[str, Sequence[str], None] = None
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    """Upgrade schema."""
    import importlib.resources
    sql_text = importlib.resources.files("src.db.sql").joinpath("001_initial.sql").read_text()
    for statement in sql_text.split(";"):
        stmt = statement.strip()
        if stmt:
            op.execute(sa.text(stmt))


def downgrade() -> None:
    """Downgrade schema."""
    op.execute("DROP TABLE IF EXISTS images")
    op.execute("DROP TABLE IF EXISTS classes")
