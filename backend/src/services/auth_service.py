"""
Authentication service for user management, password hashing, and JWT tokens.
"""
import uuid
from datetime import datetime, timezone
from typing import Optional
from sqlalchemy.orm import Session
import bcrypt
import jwt

from ..config.auth import (
    JWT_SECRET_KEY,
    JWT_ALGORITHM,
    ACCESS_TOKEN_EXPIRE_DELTA,
    REFRESH_TOKEN_EXPIRE_DELTA,
    PASSWORD_MIN_LENGTH,
    BCRYPT_ROUNDS,
)
from ..models.user import User
from ..utils.errors import AuthenticationException, ValidationException
from ..utils.logger import logger


class AuthService:
    """Service class for authentication operations."""

    @staticmethod
    def hash_password(password: str) -> str:
        """Hash a password using bcrypt."""
        password_bytes = password.encode('utf-8')
        salt = bcrypt.gensalt(rounds=BCRYPT_ROUNDS)
        hashed = bcrypt.hashpw(password_bytes, salt)
        return hashed.decode('utf-8')

    @staticmethod
    def verify_password(plain_password: str, hashed_password: str) -> bool:
        """Verify a password against its hash."""
        password_bytes = plain_password.encode('utf-8')
        hashed_bytes = hashed_password.encode('utf-8')
        return bcrypt.checkpw(password_bytes, hashed_bytes)

    @staticmethod
    def validate_password(password: str) -> None:
        """Validate password meets requirements."""
        if len(password) < PASSWORD_MIN_LENGTH:
            raise ValidationException(
                f"Password must be at least {PASSWORD_MIN_LENGTH} characters long"
            )
        if not any(c.isupper() for c in password):
            raise ValidationException("Password must contain at least one uppercase letter")
        if not any(c.islower() for c in password):
            raise ValidationException("Password must contain at least one lowercase letter")
        if not any(c.isdigit() for c in password):
            raise ValidationException("Password must contain at least one digit")

    @staticmethod
    def create_access_token(user_id: str, email: str) -> str:
        """Create a JWT access token."""
        expire = datetime.now(timezone.utc) + ACCESS_TOKEN_EXPIRE_DELTA
        payload = {
            "sub": str(user_id),
            "email": email,
            "type": "access",
            "exp": expire,
            "iat": datetime.now(timezone.utc),
        }
        return jwt.encode(payload, JWT_SECRET_KEY, algorithm=JWT_ALGORITHM)

    @staticmethod
    def create_refresh_token(user_id: str) -> str:
        """Create a JWT refresh token."""
        expire = datetime.now(timezone.utc) + REFRESH_TOKEN_EXPIRE_DELTA
        payload = {
            "sub": str(user_id),
            "type": "refresh",
            "exp": expire,
            "iat": datetime.now(timezone.utc),
        }
        return jwt.encode(payload, JWT_SECRET_KEY, algorithm=JWT_ALGORITHM)

    @staticmethod
    def decode_token(token: str) -> dict:
        """Decode and validate a JWT token."""
        try:
            payload = jwt.decode(token, JWT_SECRET_KEY, algorithms=[JWT_ALGORITHM])
            return payload
        except jwt.ExpiredSignatureError:
            raise AuthenticationException("Token has expired")
        except jwt.InvalidTokenError as e:
            raise AuthenticationException(f"Invalid token: {str(e)}")

    @staticmethod
    def create_user(
        db: Session,
        username: str,
        email: str,
        password: str,
        software_background: Optional[str] = None,
        hardware_background: Optional[str] = None,
    ) -> User:
        """Create a new user."""
        # Check if email already exists
        existing_user = db.query(User).filter(User.email == email).first()
        if existing_user:
            raise ValidationException("Email already registered", details={"field": "email"})

        # Check if username already exists
        existing_username = db.query(User).filter(User.username == username).first()
        if existing_username:
            raise ValidationException("Username already taken", details={"field": "username"})

        # Validate password
        AuthService.validate_password(password)

        # Create user
        user = User(
            id=uuid.uuid4(),
            username=username,
            email=email,
            password_hash=AuthService.hash_password(password),
            software_background=software_background,
            hardware_background=hardware_background,
        )

        db.add(user)
        db.commit()
        db.refresh(user)

        logger.info(f"User created: {user.email}")
        return user

    @staticmethod
    def authenticate_user(db: Session, email: str, password: str) -> User:
        """Authenticate a user by email and password."""
        user = db.query(User).filter(User.email == email).first()

        if not user:
            raise AuthenticationException("Invalid email or password")

        if not AuthService.verify_password(password, user.password_hash):
            raise AuthenticationException("Invalid email or password")

        logger.info(f"User authenticated: {user.email}")
        return user

    @staticmethod
    def get_user_by_id(db: Session, user_id: str) -> Optional[User]:
        """Get a user by ID."""
        try:
            user_uuid = uuid.UUID(user_id)
        except ValueError:
            return None
        return db.query(User).filter(User.id == user_uuid).first()

    @staticmethod
    def update_user(
        db: Session,
        user: User,
        username: Optional[str] = None,
        software_background: Optional[str] = None,
        hardware_background: Optional[str] = None,
        selected_language: Optional[str] = None,
        personalization_preferences: Optional[dict] = None,
    ) -> User:
        """Update user profile."""
        if username and username != user.username:
            # Check if new username is taken
            existing = db.query(User).filter(User.username == username).first()
            if existing:
                raise ValidationException("Username already taken", details={"field": "username"})
            user.username = username

        if software_background is not None:
            user.software_background = software_background
        if hardware_background is not None:
            user.hardware_background = hardware_background
        if selected_language is not None:
            user.selected_language = selected_language
        if personalization_preferences is not None:
            user.personalization_preferences = personalization_preferences

        db.commit()
        db.refresh(user)

        logger.info(f"User updated: {user.email}")
        return user

