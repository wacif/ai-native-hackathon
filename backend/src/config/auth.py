"""
Authentication configuration for FastAPI native JWT auth.
"""
import os
from datetime import timedelta
from dotenv import load_dotenv

load_dotenv()

# JWT Configuration
JWT_SECRET_KEY = os.getenv("JWT_SECRET_KEY", "your-super-secret-key-change-in-production")
JWT_ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 60  # 1 hour
REFRESH_TOKEN_EXPIRE_DAYS = 7

# Password hashing configuration
PASSWORD_MIN_LENGTH = 8
BCRYPT_ROUNDS = 12

# Token configuration
ACCESS_TOKEN_EXPIRE_DELTA = timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
REFRESH_TOKEN_EXPIRE_DELTA = timedelta(days=REFRESH_TOKEN_EXPIRE_DAYS)

# OAuth2 scheme configuration
OAUTH2_TOKEN_URL = "/auth/signin"

