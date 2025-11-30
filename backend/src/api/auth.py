"""
Authentication API endpoints for user registration, login, and profile management.
"""
from typing import Optional, List
from fastapi import APIRouter, Depends, HTTPException, status, Form
from fastapi.security import OAuth2PasswordBearer
from pydantic import BaseModel, EmailStr
from sqlalchemy.orm import Session

from ..config.db import get_db
from ..config.auth import ACCESS_TOKEN_EXPIRE_MINUTES
from ..services.auth_service import AuthService
from ..utils.errors import AuthenticationException, ValidationException
from ..utils.logger import logger

router = APIRouter(prefix="/auth", tags=["Authentication"])

# OAuth2 scheme for token authentication
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="/auth/signin")


# ============ Request/Response Models ============

class SignupRequest(BaseModel):
    username: str
    email: EmailStr
    password: str
    software_background: Optional[str] = None
    hardware_background: Optional[str] = None
    # New personalization fields
    programming_languages: Optional[List[str]] = None
    operating_system: Optional[str] = None
    learning_goals: Optional[List[str]] = None
    preferred_explanation_style: Optional[str] = None
    prior_knowledge: Optional[List[str]] = None
    industry: Optional[str] = None


class SignupResponse(BaseModel):
    message: str
    user_id: str


class SigninResponse(BaseModel):
    message: str
    access_token: str
    refresh_token: str
    token_type: str = "bearer"
    expires_in: int = ACCESS_TOKEN_EXPIRE_MINUTES * 60  # seconds


class RefreshRequest(BaseModel):
    refresh_token: str


class RefreshResponse(BaseModel):
    access_token: str
    token_type: str = "bearer"
    expires_in: int = ACCESS_TOKEN_EXPIRE_MINUTES * 60


class UserProfileResponse(BaseModel):
    user_id: str
    username: str
    email: str
    software_background: Optional[str]
    hardware_background: Optional[str]
    personalization_preferences: Optional[dict]
    selected_language: str
    # New personalization fields
    programming_languages: Optional[List[str]]
    operating_system: Optional[str]
    learning_goals: Optional[List[str]]
    preferred_explanation_style: Optional[str]
    prior_knowledge: Optional[List[str]]
    industry: Optional[str]
    created_at: str
    updated_at: str


class UpdateProfileRequest(BaseModel):
    username: Optional[str] = None
    software_background: Optional[str] = None
    hardware_background: Optional[str] = None
    selected_language: Optional[str] = None
    personalization_preferences: Optional[dict] = None
    # New personalization fields
    programming_languages: Optional[List[str]] = None
    operating_system: Optional[str] = None
    learning_goals: Optional[List[str]] = None
    preferred_explanation_style: Optional[str] = None
    prior_knowledge: Optional[List[str]] = None
    industry: Optional[str] = None


class UpdateProfileResponse(BaseModel):
    message: str
    user: UserProfileResponse


# ============ Dependencies ============

async def get_current_user(
    token: str = Depends(oauth2_scheme),
    db: Session = Depends(get_db)
):
    """Dependency to get the current authenticated user from JWT token."""
    try:
        payload = AuthService.decode_token(token)
        user_id = payload.get("sub")
        token_type = payload.get("type")

        if not user_id or token_type != "access":
            raise AuthenticationException("Invalid token")

        user = AuthService.get_user_by_id(db, user_id)
        if not user:
            raise AuthenticationException("User not found")

        return user
    except AuthenticationException:
        raise
    except Exception as e:
        logger.error(f"Auth error: {str(e)}")
        raise AuthenticationException("Could not validate credentials")


# ============ Endpoints ============

@router.post("/signup", response_model=SignupResponse, status_code=status.HTTP_201_CREATED)
async def signup(request: SignupRequest, db: Session = Depends(get_db)):
    """
    Register a new user.
    
    - **username**: Unique username (required)
    - **email**: Valid email address (required)
    - **password**: Minimum 8 characters with uppercase, lowercase, and digit (required)
    - **software_background**: User's software experience level (optional)
    - **hardware_background**: User's hardware experience level (optional)
    - **programming_languages**: List of programming languages user knows (optional)
    - **operating_system**: User's primary OS - windows/macos/linux (optional)
    - **learning_goals**: What the user wants to learn (optional)
    - **preferred_explanation_style**: How the user prefers content - conceptual/code-heavy/visual/step-by-step (optional)
    - **prior_knowledge**: Topics the user already knows (optional)
    - **industry**: User's field - student/researcher/industry/hobbyist (optional)
    """
    try:
        user = AuthService.create_user(
            db=db,
            username=request.username,
            email=request.email,
            password=request.password,
            software_background=request.software_background,
            hardware_background=request.hardware_background,
            programming_languages=request.programming_languages,
            operating_system=request.operating_system,
            learning_goals=request.learning_goals,
            preferred_explanation_style=request.preferred_explanation_style,
            prior_knowledge=request.prior_knowledge,
            industry=request.industry,
        )
        return SignupResponse(
            message="User registered successfully",
            user_id=str(user.id)
        )
    except ValidationException as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=e.message
        )


@router.post("/signin", response_model=SigninResponse)
async def signin(
    username: str = Form(..., description="Email address"),
    password: str = Form(...),
    db: Session = Depends(get_db)
):
    """
    Authenticate user and return JWT tokens.
    
    Uses OAuth2 password flow - send credentials as form data:
    - **username**: Email address (OAuth2 convention uses 'username' field)
    - **password**: User's password
    """
    try:
        # OAuth2 form uses 'username' field, but we accept email
        user = AuthService.authenticate_user(db, username, password)

        access_token = AuthService.create_access_token(str(user.id), user.email)
        refresh_token = AuthService.create_refresh_token(str(user.id))

        return SigninResponse(
            message="Login successful",
            access_token=access_token,
            refresh_token=refresh_token,
        )
    except AuthenticationException as e:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail=e.message,
            headers={"WWW-Authenticate": "Bearer"},
        )


@router.post("/refresh", response_model=RefreshResponse)
async def refresh_token(request: RefreshRequest, db: Session = Depends(get_db)):
    """
    Refresh an expired access token using a valid refresh token.
    """
    try:
        payload = AuthService.decode_token(request.refresh_token)
        user_id = payload.get("sub")
        token_type = payload.get("type")

        if not user_id or token_type != "refresh":
            raise AuthenticationException("Invalid refresh token")

        user = AuthService.get_user_by_id(db, user_id)
        if not user:
            raise AuthenticationException("User not found")

        new_access_token = AuthService.create_access_token(str(user.id), user.email)

        return RefreshResponse(access_token=new_access_token)
    except AuthenticationException as e:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail=e.message,
            headers={"WWW-Authenticate": "Bearer"},
        )


@router.get("/me", response_model=UserProfileResponse)
async def get_profile(current_user = Depends(get_current_user)):
    """
    Get the current authenticated user's profile.
    
    Requires: Bearer token in Authorization header
    """
    return UserProfileResponse(
        user_id=str(current_user.id),
        username=current_user.username,
        email=current_user.email,
        software_background=current_user.software_background,
        hardware_background=current_user.hardware_background,
        personalization_preferences=current_user.personalization_preferences,
        selected_language=current_user.selected_language,
        programming_languages=current_user.programming_languages or [],
        operating_system=current_user.operating_system,
        learning_goals=current_user.learning_goals or [],
        preferred_explanation_style=current_user.preferred_explanation_style,
        prior_knowledge=current_user.prior_knowledge or [],
        industry=current_user.industry,
        created_at=current_user.created_at.isoformat(),
        updated_at=current_user.updated_at.isoformat(),
    )


@router.patch("/me", response_model=UpdateProfileResponse)
async def update_profile(
    request: UpdateProfileRequest,
    current_user = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """
    Update the current authenticated user's profile.
    
    All fields are optional - only provided fields will be updated.
    """
    try:
        updated_user = AuthService.update_user(
            db=db,
            user=current_user,
            username=request.username,
            software_background=request.software_background,
            hardware_background=request.hardware_background,
            selected_language=request.selected_language,
            personalization_preferences=request.personalization_preferences,
            programming_languages=request.programming_languages,
            operating_system=request.operating_system,
            learning_goals=request.learning_goals,
            preferred_explanation_style=request.preferred_explanation_style,
            prior_knowledge=request.prior_knowledge,
            industry=request.industry,
        )

        return UpdateProfileResponse(
            message="Profile updated successfully",
            user=UserProfileResponse(
                user_id=str(updated_user.id),
                username=updated_user.username,
                email=updated_user.email,
                software_background=updated_user.software_background,
                hardware_background=updated_user.hardware_background,
                personalization_preferences=updated_user.personalization_preferences,
                selected_language=updated_user.selected_language,
                programming_languages=updated_user.programming_languages or [],
                operating_system=updated_user.operating_system,
                learning_goals=updated_user.learning_goals or [],
                preferred_explanation_style=updated_user.preferred_explanation_style,
                prior_knowledge=updated_user.prior_knowledge or [],
                industry=updated_user.industry,
                created_at=updated_user.created_at.isoformat(),
                updated_at=updated_user.updated_at.isoformat(),
            )
        )
    except ValidationException as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=e.message
        )

