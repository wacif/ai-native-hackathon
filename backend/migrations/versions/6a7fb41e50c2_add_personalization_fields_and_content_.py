"""add_personalization_fields_and_content_table

Revision ID: 6a7fb41e50c2
Revises: 65face620c30
Create Date: 2025-11-30 13:06:12.639220

"""
from typing import Sequence, Union

from alembic import op
import sqlalchemy as sa


# revision identifiers, used by Alembic.
revision: str = '6a7fb41e50c2'
down_revision: Union[str, Sequence[str], None] = '65face620c30'
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    """Upgrade schema."""
    pass


def downgrade() -> None:
    """Downgrade schema."""
    pass
