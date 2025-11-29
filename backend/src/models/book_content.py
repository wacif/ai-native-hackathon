import uuid
from sqlalchemy import Column, String, Text, DateTime, Integer, func, JSONB, ForeignKey
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import relationship

from backend.src.models.base import Base

class BookContent(Base):
    __tablename__ = "book_content"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    title = Column(String, nullable=False)
    raw_text = Column(Text, nullable=False)
    personalized_text = Column(JSONB, nullable=True)
    urdu_text = Column(Text, nullable=True)
    module_id = Column(UUID(as_uuid=True), ForeignKey("modules.id"), nullable=True) # Assuming a Module entity
    order = Column(Integer, nullable=False)
    created_at = Column(DateTime, default=func.now(), nullable=False)
    updated_at = Column(DateTime, default=func.now(), onupdate=func.now(), nullable=False)

    # Optional relationship to a Module if it becomes a separate entity
    # module = relationship("Module", back_populates="content")

    def __repr__(self):
        return f"<BookContent(id={self.id}, title='{self.title}')>"
