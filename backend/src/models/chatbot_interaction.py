from sqlalchemy import Column, String, Text, DateTime, ForeignKey, Boolean
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
import uuid
from .base import Base

class ChatbotInteraction(Base):
    __tablename__ = "chatbot_interactions"

    interaction_id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id"), nullable=False)
    query_text = Column(Text, nullable=False)
    response_text = Column(Text, nullable=False)
    timestamp = Column(DateTime(timezone=True), server_default=func.now())
    is_feedback_positive = Column(Boolean, nullable=True) # User feedback on response

    user = relationship("User", backref="chatbot_interactions")
