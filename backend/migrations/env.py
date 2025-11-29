import sys
import os
from logging.config import fileConfig

from sqlalchemy import engine_from_config
from sqlalchemy import pool

from alembic import context

# This ensures that your models can be found by Alembic
sys.path.append(os.getcwd())
from src.models.user import Base as UserBase
from src.models.book_content import Base as BookContentBase
from src.models.chatbot_interaction import Base as ChatbotInteractionBase

# Interpret the config file for Python's logging.
# This line sets up loggers basically.
fileConfig(context.config.config_file_name)

# add your model's MetaData object here
# for 'autogenerate' support
# from myapp import mymodel
# target_metadata = mymodel.Base.metadata
target_metadata = [UserBase.metadata, BookContentBase.metadata, ChatbotInteractionBase.metadata]

# other values from the config, defined by the needs of env.py,
# can be acquired: copy from the config above.
# my_important_option = config.get_main_option("my_important_option")
# ... etc.

def run_migrations_offline():
    """Run migrations in 'offline' mode.

    This configures the context with just a URL
    and not an Engine, though an Engine is acceptable
    here as well.  By skipping the Engine creation
    we don't even need a database to begin with.

    Calls to context.execute() here emit the given string to the
    script output.

    """
    url = context.config.get_main_option("sqlalchemy.url")
    context.configure(
        url=url,
        target_metadata=target_metadata,
        literal_binds=True,
        dialect_opts={"paramstyle": "sqlalchemy"},
    )

    with context.begin_transaction():
        context.run_migrations()

def run_migrations_online():
    """Run migrations in 'online' mode.

    In this scenario we need to create an Engine
    and associate a connection with the context.

    """
    configuration = context.config.get_section(context.config.config_ini_section)
    configuration["sqlalchemy.url"] = os.getenv("DATABASE_URL") # Ensure this is set from environment
    connectable = engine_from_config(
        configuration,
        prefix="sqlalchemy.",
        poolclass=pool.NullPool,
    )

    with connectable.connect() as connection:
        context.configure(
            connection=connection,
            target_metadata=target_metadata,
        )

        with context.begin_transaction():
            context.run_migrations()

if context.is_offline_mode():
    run_migrations_offline()
elif os.getenv("DATABASE_URL") is None:
    raise Exception("DATABASE_URL environment variable is not set. Please set it to proceed with migrations.")
else:
    run_migrations_online()
