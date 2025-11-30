"""add_personalization_fields_and_content_table

Revision ID: e7f97eb76c81
Revises: 6a7fb41e50c2
Create Date: 2025-11-30 13:06:20.470776

"""
from typing import Sequence, Union

from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects.postgresql import UUID, JSONB


# revision identifiers, used by Alembic.
revision: str = 'e7f97eb76c81'
down_revision: Union[str, Sequence[str], None] = '6a7fb41e50c2'
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    """Upgrade schema."""
    # Add new columns to users table for personalization
    op.add_column('users', sa.Column('programming_languages', JSONB, nullable=True, server_default='[]'))
    op.add_column('users', sa.Column('operating_system', sa.String(20), nullable=True))
    op.add_column('users', sa.Column('learning_goals', JSONB, nullable=True, server_default='[]'))
    op.add_column('users', sa.Column('preferred_explanation_style', sa.String(30), nullable=True))
    op.add_column('users', sa.Column('prior_knowledge', JSONB, nullable=True, server_default='[]'))
    op.add_column('users', sa.Column('industry', sa.String(50), nullable=True))

    # Create personalized_content table
    op.create_table(
        'personalized_content',
        sa.Column('id', UUID(as_uuid=True), primary_key=True, server_default=sa.text('gen_random_uuid()')),
        sa.Column('user_id', UUID(as_uuid=True), sa.ForeignKey('users.id', ondelete='CASCADE'), nullable=False),
        sa.Column('chapter_id', sa.String(100), nullable=False),
        sa.Column('original_content_hash', sa.String(64), nullable=True),
        sa.Column('personalized_content', sa.Text, nullable=False),
        sa.Column('created_at', sa.DateTime, server_default=sa.func.now(), nullable=False),
        sa.Column('updated_at', sa.DateTime, server_default=sa.func.now(), onupdate=sa.func.now(), nullable=False),
        sa.UniqueConstraint('user_id', 'chapter_id', name='uq_user_chapter')
    )

    # Create index for faster lookups
    op.create_index('idx_personalized_user_chapter', 'personalized_content', ['user_id', 'chapter_id'])


def downgrade() -> None:
    """Downgrade schema."""
    # Drop personalized_content table
    op.drop_index('idx_personalized_user_chapter', table_name='personalized_content')
    op.drop_table('personalized_content')

    # Remove new columns from users table
    op.drop_column('users', 'industry')
    op.drop_column('users', 'prior_knowledge')
    op.drop_column('users', 'preferred_explanation_style')
    op.drop_column('users', 'learning_goals')
    op.drop_column('users', 'operating_system')
    op.drop_column('users', 'programming_languages')
