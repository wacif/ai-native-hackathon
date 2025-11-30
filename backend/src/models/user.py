import uuid
from sqlalchemy import Column, String, DateTime, func
from sqlalchemy.dialects.postgresql import UUID, JSONB

from .base import Base

class User(Base):
    __tablename__ = "users"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    username = Column(String, unique=True, index=True, nullable=False)
    email = Column(String, unique=True, index=True, nullable=False)
    password_hash = Column(String, nullable=False)
    software_background = Column(String, nullable=True)
    hardware_background = Column(String, nullable=True)
    personalization_preferences = Column(JSONB, nullable=True)
    selected_language = Column(String, default="en", nullable=False)
    
    # New personalization fields
    programming_languages = Column(JSONB, nullable=True, default=list)  # ["Python", "C++", "JavaScript"]
    operating_system = Column(String(20), nullable=True)  # "windows" | "macos" | "linux"
    learning_goals = Column(JSONB, nullable=True, default=list)  # ["ROS2", "Isaac Sim", "Robot Control"]
    preferred_explanation_style = Column(String(30), nullable=True)  # "conceptual" | "code-heavy" | "visual" | "step-by-step"
    prior_knowledge = Column(JSONB, nullable=True, default=list)  # ["Python basics", "Linux", "Control theory"]
    industry = Column(String(50), nullable=True)  # "student" | "researcher" | "industry" | "hobbyist"
    
    created_at = Column(DateTime, default=func.now(), nullable=False)
    updated_at = Column(DateTime, default=func.now(), onupdate=func.now(), nullable=False)

    def __repr__(self):
        return f"<User(id={self.id}, username='{self.username}', email='{self.email}')>"
