# Data Models: Physical AI & Humanoid Robotics Book Platform Backend

## Overview
This document defines all data models required for the Physical AI & Humanoid Robotics book platform backend, following the project constitution requirements.

## User Model (Optional)
**Purpose**: Represents platform users with optional authentication and authorization capabilities (for enhanced features)

### SQLAlchemy Model
```python
class User(Base):
    __tablename__ = "users"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, index=True)
    email: Mapped[str] = mapped_column(String, unique=True, index=True, nullable=False)
    hashed_password: Mapped[str] = mapped_column(String, nullable=False)
    full_name: Mapped[Optional[str]] = mapped_column(String)
    is_active: Mapped[bool] = mapped_column(Boolean, default=True)
    is_admin: Mapped[bool] = mapped_column(Boolean, default=False)
    created_at: Mapped[datetime] = mapped_column(DateTime, default=func.now())
    updated_at: Mapped[datetime] = mapped_column(DateTime, default=func.now(), onupdate=func.now())

    # Relationships
    notes: Mapped[List["Note"]] = relationship("Note", back_populates="user")
    highlights: Mapped[List["Highlight"]] = relationship("Highlight", back_populates="user")
    bookmarks: Mapped[List["Bookmark"]] = relationship("Bookmark", back_populates="user")
    chat_histories: Mapped[List["ChatHistory"]] = relationship("ChatHistory", back_populates="user")
    rag_queries: Mapped[List["RAGQuery"]] = relationship("RAGQuery", back_populates="user")
    api_usage_quotas: Mapped[List["APIUsageQuota"]] = relationship("APIUsageQuota", back_populates="user")
```

### Pydantic Schemas
```python
class UserBase(BaseModel):
    email: EmailStr
    full_name: Optional[str] = None
    is_active: Optional[bool] = True

class UserCreate(UserBase):
    password: str

class UserUpdate(UserBase):
    password: Optional[str] = None

class UserInDB(UserBase):
    id: int
    is_admin: bool
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True
```

## Chapter Model
**Purpose**: Represents book chapters with metadata and ordering

### SQLAlchemy Model
```python
class Chapter(Base):
    __tablename__ = "chapters"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, index=True)
    title: Mapped[str] = mapped_column(String, nullable=False)
    description: Mapped[Optional[str]] = mapped_column(String)
    order_index: Mapped[int] = mapped_column(Integer, nullable=False)
    module_id: Mapped[int] = mapped_column(Integer, ForeignKey("modules.id"), nullable=False)
    created_at: Mapped[datetime] = mapped_column(DateTime, default=func.now())
    updated_at: Mapped[datetime] = mapped_column(DateTime, default=func.now(), onupdate=func.now())

    # Relationships
    module: Mapped["Module"] = relationship("Module", back_populates="chapters")
    lessons: Mapped[List["Lesson"]] = relationship("Lesson", back_populates="chapter")
    embedding_documents: Mapped[List["EmbeddingDocument"]] = relationship("EmbeddingDocument", back_populates="chapter")
```

### Pydantic Schemas
```python
class ChapterBase(BaseModel):
    title: str
    description: Optional[str] = None
    order_index: int
    module_id: int

class ChapterCreate(ChapterBase):
    pass

class ChapterUpdate(ChapterBase):
    title: Optional[str] = None
    order_index: Optional[int] = None

class ChapterInDB(ChapterBase):
    id: int
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True
```

## Lesson Model
**Purpose**: Represents individual lessons within chapters

### SQLAlchemy Model
```python
class Lesson(Base):
    __tablename__ = "lessons"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, index=True)
    title: Mapped[str] = mapped_column(String, nullable=False)
    description: Mapped[Optional[str]] = mapped_column(String)
    content: Mapped[Optional[str]] = mapped_column(Text)  # Markdown content
    order_index: Mapped[int] = mapped_column(Integer, nullable=False)
    chapter_id: Mapped[int] = mapped_column(Integer, ForeignKey("chapters.id"), nullable=False)
    created_at: Mapped[datetime] = mapped_column(DateTime, default=func.now())
    updated_at: Mapped[datetime] = mapped_column(DateTime, default=func.now(), onupdate=func.now())

    # Relationships
    chapter: Mapped["Chapter"] = relationship("Chapter", back_populates="lessons")
    lesson_sections: Mapped[List["LessonSection"]] = relationship("LessonSection", back_populates="lesson")
    embedding_documents: Mapped[List["EmbeddingDocument"]] = relationship("EmbeddingDocument", back_populates="lesson")
    user_progress: Mapped[List["UserProgress"]] = relationship("UserProgress", back_populates="lesson")
```

### Pydantic Schemas
```python
class LessonBase(BaseModel):
    title: str
    description: Optional[str] = None
    content: Optional[str] = None
    order_index: int
    chapter_id: int

class LessonCreate(LessonBase):
    pass

class LessonUpdate(LessonBase):
    title: Optional[str] = None
    order_index: Optional[int] = None

class LessonInDB(LessonBase):
    id: int
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True
```

## LessonSection Model
**Purpose**: Represents smaller sections within lessons for better content organization

### SQLAlchemy Model
```python
class LessonSection(Base):
    __tablename__ = "lesson_sections"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, index=True)
    title: Mapped[str] = mapped_column(String, nullable=False)
    content: Mapped[str] = mapped_column(Text, nullable=False)  # Markdown content
    order_index: Mapped[int] = mapped_column(Integer, nullable=False)
    lesson_id: Mapped[int] = mapped_column(Integer, ForeignKey("lessons.id"), nullable=False)
    created_at: Mapped[datetime] = mapped_column(DateTime, default=func.now())
    updated_at: Mapped[datetime] = mapped_column(DateTime, default=func.now(), onupdate=func.now())

    # Relationships
    lesson: Mapped["Lesson"] = relationship("Lesson", back_populates="lesson_sections")
    embedding_documents: Mapped[List["EmbeddingDocument"]] = relationship("EmbeddingDocument", back_populates="lesson_section")
```

### Pydantic Schemas
```python
class LessonSectionBase(BaseModel):
    title: str
    content: str
    order_index: int
    lesson_id: int

class LessonSectionCreate(LessonSectionBase):
    pass

class LessonSectionUpdate(LessonSectionBase):
    title: Optional[str] = None
    content: Optional[str] = None
    order_index: Optional[int] = None

class LessonSectionInDB(LessonSectionBase):
    id: int
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True
```

## EmbeddingDocument Model
**Purpose**: Links content chunks to vector embeddings in Qdrant with metadata

### SQLAlchemy Model
```python
class EmbeddingDocument(Base):
    __tablename__ = "embedding_documents"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, index=True)
    qdrant_id: Mapped[str] = mapped_column(String, unique=True, nullable=False)  # ID in Qdrant
    content: Mapped[str] = mapped_column(Text, nullable=False)  # The actual content chunk
    content_type: Mapped[str] = mapped_column(String, nullable=False)  # "lesson", "lesson_section", etc.
    content_id: Mapped[int] = mapped_column(Integer, nullable=False)  # ID of the content in its table
    metadata: Mapped[Dict] = mapped_column(JSON, nullable=False)  # Additional metadata for search
    embedding_model: Mapped[str] = mapped_column(String, default="text-embedding-3-small")
    tokens: Mapped[int] = mapped_column(Integer, nullable=False)
    created_at: Mapped[datetime] = mapped_column(DateTime, default=func.now())
    updated_at: Mapped[datetime] = mapped_column(DateTime, default=func.now(), onupdate=func.now())

    # Relationships
    chapter: Mapped[Optional["Chapter"]] = relationship("Chapter", back_populates="embedding_documents")
    lesson: Mapped[Optional["Lesson"]] = relationship("Lesson", back_populates="embedding_documents")
    lesson_section: Mapped[Optional["LessonSection"]] = relationship("LessonSection", back_populates="embedding_documents")

    # Foreign key references (stored as metadata but linked to actual content)
    chapter_id: Mapped[Optional[int]] = mapped_column(Integer, ForeignKey("chapters.id"))
    lesson_id: Mapped[Optional[int]] = mapped_column(Integer, ForeignKey("lessons.id"))
    lesson_section_id: Mapped[Optional[int]] = mapped_column(Integer, ForeignKey("lesson_sections.id"))
```

### Pydantic Schemas
```python
class EmbeddingDocumentBase(BaseModel):
    qdrant_id: str
    content: str
    content_type: str
    content_id: int
    metadata: Dict[str, Any]
    embedding_model: str = "text-embedding-3-small"
    tokens: int

class EmbeddingDocumentCreate(EmbeddingDocumentBase):
    pass

class EmbeddingDocumentUpdate(BaseModel):
    metadata: Optional[Dict[str, Any]] = None

class EmbeddingDocumentInDB(EmbeddingDocumentBase):
    id: int
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True
```

## RAGQuery Model
**Purpose**: Tracks user RAG queries for analytics and improvement

### SQLAlchemy Model
```python
class RAGQuery(Base):
    __tablename__ = "rag_queries"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, index=True)
    user_id: Mapped[int] = mapped_column(Integer, ForeignKey("users.id"), nullable=False)
    query_text: Mapped[str] = mapped_column(Text, nullable=False)
    response_text: Mapped[str] = mapped_column(Text, nullable=False)
    context_chunks: Mapped[List[Dict]] = mapped_column(ARRAY(JSON), nullable=False)  # Retrieved chunks
    tokens_used: Mapped[int] = mapped_column(Integer, nullable=False)
    response_time_ms: Mapped[int] = mapped_column(Integer, nullable=False)
    is_hallucination: Mapped[bool] = mapped_column(Boolean, default=False)  # Flag for quality control
    feedback_score: Mapped[Optional[int]] = mapped_column(Integer)  # 1-5 rating
    created_at: Mapped[datetime] = mapped_column(DateTime, default=func.now())

    # Relationships
    user: Mapped["User"] = relationship("User", back_populates="rag_queries")
```

### Pydantic Schemas
```python
class RAGQueryBase(BaseModel):
    user_id: int
    query_text: str
    response_text: str
    context_chunks: List[Dict[str, Any]]
    tokens_used: int
    response_time_ms: int
    is_hallucination: Optional[bool] = False
    feedback_score: Optional[int] = None

class RAGQueryCreate(RAGQueryBase):
    pass

class RAGQueryUpdate(BaseModel):
    feedback_score: Optional[int] = None
    is_hallucination: Optional[bool] = None

class RAGQueryInDB(RAGQueryBase):
    id: int
    created_at: datetime

    class Config:
        from_attributes = True
```

## ChatHistory Model
**Purpose**: Stores conversation history for users

### SQLAlchemy Model
```python
class ChatHistory(Base):
    __tablename__ = "chat_history"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, index=True)
    user_id: Mapped[int] = mapped_column(Integer, ForeignKey("users.id"), nullable=False)
    session_id: Mapped[str] = mapped_column(String, index=True, nullable=False)
    role: Mapped[str] = mapped_column(String, nullable=False)  # "user" or "assistant"
    content: Mapped[str] = mapped_column(Text, nullable=False)
    timestamp: Mapped[datetime] = mapped_column(DateTime, default=func.now())

    # Relationships
    user: Mapped["User"] = relationship("User", back_populates="chat_histories")
```

### Pydantic Schemas
```python
class ChatHistoryBase(BaseModel):
    user_id: int
    session_id: str
    role: str  # "user" or "assistant"
    content: str

class ChatHistoryCreate(ChatHistoryBase):
    pass

class ChatHistoryInDB(ChatHistoryBase):
    id: int
    timestamp: datetime

    class Config:
        from_attributes = True
```

## Highlight Model (Optional)
**Purpose**: Stores user highlights in lessons (optional feature)

### SQLAlchemy Model
```python
class Highlight(Base):
    __tablename__ = "highlights"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, index=True)
    user_id: Mapped[int] = mapped_column(Integer, ForeignKey("users.id"), nullable=False)
    content_id: Mapped[int] = mapped_column(Integer, nullable=False)  # ID of the content (lesson/section)
    content_type: Mapped[str] = mapped_column(String, nullable=False)  # "lesson" or "lesson_section"
    text: Mapped[str] = mapped_column(Text, nullable=False)  # The highlighted text
    start_pos: Mapped[int] = mapped_column(Integer, nullable=False)  # Start position in content
    end_pos: Mapped[int] = mapped_column(Integer, nullable=False)  # End position in content
    note: Mapped[Optional[str]] = mapped_column(String)  # Optional note about the highlight
    created_at: Mapped[datetime] = mapped_column(DateTime, default=func.now())
    updated_at: Mapped[datetime] = mapped_column(DateTime, default=func.now(), onupdate=func.now())

    # Relationships
    user: Mapped["User"] = relationship("User", back_populates="highlights")
```

### Pydantic Schemas
```python
class HighlightBase(BaseModel):
    user_id: int
    content_id: int
    content_type: str
    text: str
    start_pos: int
    end_pos: int
    note: Optional[str] = None

class HighlightCreate(HighlightBase):
    pass

class HighlightUpdate(BaseModel):
    note: Optional[str] = None

class HighlightInDB(HighlightBase):
    id: int
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True
```

## Note Model (Optional)
**Purpose**: Stores user notes associated with content (optional feature)

### SQLAlchemy Model
```python
class Note(Base):
    __tablename__ = "notes"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, index=True)
    user_id: Mapped[int] = mapped_column(Integer, ForeignKey("users.id"), nullable=False)
    content_id: Mapped[int] = mapped_column(Integer, nullable=False)  # ID of the content (lesson/section)
    content_type: Mapped[str] = mapped_column(String, nullable=False)  # "lesson" or "lesson_section"
    title: Mapped[str] = mapped_column(String, nullable=False)
    content: Mapped[str] = mapped_column(Text, nullable=False)  # The note content
    is_public: Mapped[bool] = mapped_column(Boolean, default=False)  # Whether note is visible to others
    created_at: Mapped[datetime] = mapped_column(DateTime, default=func.now())
    updated_at: Mapped[datetime] = mapped_column(DateTime, default=func.now(), onupdate=func.now())

    # Relationships
    user: Mapped["User"] = relationship("User", back_populates="notes")
```

### Pydantic Schemas
```python
class NoteBase(BaseModel):
    user_id: int
    content_id: int
    content_type: str
    title: str
    content: str
    is_public: Optional[bool] = False

class NoteCreate(NoteBase):
    pass

class NoteUpdate(BaseModel):
    title: Optional[str] = None
    content: Optional[str] = None
    is_public: Optional[bool] = None

class NoteInDB(NoteBase):
    id: int
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True
```

## APIUsageQuota Model (Optional)
**Purpose**: Tracks API usage for users to enforce resource limits (optional feature)

### SQLAlchemy Model
```python
class APIUsageQuota(Base):
    __tablename__ = "api_usage_quotas"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, index=True)
    user_id: Mapped[int] = mapped_column(Integer, ForeignKey("users.id"), nullable=False)
    quota_type: Mapped[str] = mapped_column(String, nullable=False)  # "rag_query", "search", etc.
    quota_limit: Mapped[int] = mapped_column(Integer, nullable=False)  # Max allowed per period
    quota_used: Mapped[int] = mapped_column(Integer, default=0)  # Used in current period
    quota_period_start: Mapped[datetime] = mapped_column(DateTime, nullable=False)  # Period start time
    reset_interval_hours: Mapped[int] = mapped_column(Integer, default=24)  # Hours between resets
    created_at: Mapped[datetime] = mapped_column(DateTime, default=func.now())
    updated_at: Mapped[datetime] = mapped_column(DateTime, default=func.now(), onupdate=func.now())

    # Relationships
    user: Mapped["User"] = relationship("User", back_populates="api_usage_quotas")
```

### Pydantic Schemas
```python
class APIUsageQuotaBase(BaseModel):
    user_id: int
    quota_type: str
    quota_limit: int
    quota_used: int
    quota_period_start: datetime
    reset_interval_hours: int

class APIUsageQuotaCreate(APIUsageQuotaBase):
    pass

class APIUsageQuotaUpdate(BaseModel):
    quota_used: Optional[int] = None
    quota_period_start: Optional[datetime] = None

class APIUsageQuotaInDB(APIUsageQuotaBase):
    id: int
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True
```

## IngestionJob Model
**Purpose**: Tracks document ingestion jobs with status and results (admin/restricted feature)

### SQLAlchemy Model
```python
class IngestionJob(Base):
    __tablename__ = "ingestion_jobs"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, index=True)
    job_id: Mapped[str] = mapped_column(String, unique=True, nullable=False)  # UUID for the job
    admin_user_id: Mapped[int] = mapped_column(Integer, ForeignKey("users.id"), nullable=False)  # Who started the job
    file_path: Mapped[str] = mapped_column(String, nullable=False)  # Path to uploaded file
    file_name: Mapped[str] = mapped_column(String, nullable=False)
    file_type: Mapped[str] = mapped_column(String, nullable=False)  # "markdown", "html", "pdf", etc.
    status: Mapped[str] = mapped_column(String, default="pending")  # "pending", "processing", "completed", "failed"
    total_chunks: Mapped[Optional[int]] = mapped_column(Integer)  # Total chunks processed
    processed_chunks: Mapped[int] = mapped_column(Integer, default=0)  # Chunks completed
    error_message: Mapped[Optional[str]] = mapped_column(Text)  # Error if job failed
    started_at: Mapped[datetime] = mapped_column(DateTime, default=func.now())
    completed_at: Mapped[Optional[datetime]] = mapped_column(DateTime)

    # Relationships
    admin_user: Mapped["User"] = relationship("User", back_populates="ingestion_jobs")
```

### Pydantic Schemas
```python
class IngestionJobBase(BaseModel):
    job_id: str
    admin_user_id: int
    file_path: str
    file_name: str
    file_type: str
    status: str
    total_chunks: Optional[int] = None
    processed_chunks: int
    error_message: Optional[str] = None
    started_at: datetime
    completed_at: Optional[datetime] = None

class IngestionJobCreate(IngestionJobBase):
    pass

class IngestionJobUpdate(BaseModel):
    status: Optional[str] = None
    total_chunks: Optional[int] = None
    processed_chunks: Optional[int] = None
    error_message: Optional[str] = None
    completed_at: Optional[datetime] = None

class IngestionJobInDB(IngestionJobBase):
    id: int

    class Config:
        from_attributes = True
```

## BookMetadata Model
**Purpose**: Stores metadata about the book itself

### SQLAlchemy Model
```python
class BookMetadata(Base):
    __tablename__ = "book_metadata"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, index=True)
    title: Mapped[str] = mapped_column(String, nullable=False)
    description: Mapped[Optional[str]] = mapped_column(String)
    version: Mapped[str] = mapped_column(String, nullable=False)  # Version of the book content
    total_chapters: Mapped[int] = mapped_column(Integer, default=0)
    total_lessons: Mapped[int] = mapped_column(Integer, default=0)
    total_words: Mapped[int] = mapped_column(Integer, default=0)
    language: Mapped[str] = mapped_column(String, default="en")
    created_at: Mapped[datetime] = mapped_column(DateTime, default=func.now())
    updated_at: Mapped[datetime] = mapped_column(DateTime, default=func.now(), onupdate=func.now())
```

### Pydantic Schemas
```python
class BookMetadataBase(BaseModel):
    title: str
    description: Optional[str] = None
    version: str
    total_chapters: int
    total_lessons: int
    total_words: int
    language: str

class BookMetadataCreate(BookMetadataBase):
    pass

class BookMetadataUpdate(BaseModel):
    description: Optional[str] = None
    version: Optional[str] = None
    total_chapters: Optional[int] = None
    total_lessons: Optional[int] = None
    total_words: Optional[int] = None

class BookMetadataInDB(BookMetadataBase):
    id: int
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True
```

## Translation Feature Data Models

### TranslationRequest
**Purpose**: Request model for translating content to Urdu

```python
class TranslationRequest(BaseModel):
    content: str
        description: "The English content to be translated to Urdu"
        validation: "Required, minimum 1 character, maximum 10000 characters"

    source_language: str = "en"
        description: "Source language code (default: en for English)"
        validation: "Optional, defaults to 'en'"

    target_language: str = "ur"
        description: "Target language code (default: ur for Urdu)"
        validation: "Optional, defaults to 'ur'"
```

### TranslationResponse
**Purpose**: Response model for translation API

```python
class TranslationResponse(BaseModel):
    translated_content: str
        description: "The content translated to Urdu"
        validation: "Required"

    source_content: str
        description: "Original English content that was translated"
        validation: "Required"

    source_language: str
        description: "Source language code"
        validation: "Required, e.g., 'en'"

    target_language: str
        description: "Target language code"
        validation: "Required, e.g., 'ur'"

    translation_time: float
        description: "Time taken for translation in seconds"
        validation: "Optional, for performance monitoring"

    success: bool
        description: "Whether the translation was successful"
        validation: "Required, boolean"

    error_message: Optional[str]
        description: "Error message if translation failed"
        validation: "Optional"
```

### TranslationHistory (Optional Future Enhancement)
**Purpose**: Model for storing translation history if needed for caching or analytics

```python
class TranslationHistory(BaseModel):
    id: UUID
        description: "Unique identifier for the translation record"
        validation: "Required, auto-generated UUID"

    user_id: Optional[UUID]
        description: "ID of the user who requested translation (if authenticated)"
        validation: "Optional"

    original_content_hash: str
        description: "SHA-256 hash of the original content for deduplication"
        validation: "Required, unique"

    source_content: str
        description: "Original English content"
        validation: "Required"

    translated_content: str
        description: "Translated Urdu content"
        validation: "Required"

    source_language: str
        description: "Source language code"
        validation: "Required"

    target_language: str
        description: "Target language code"
        validation: "Required"

    created_at: datetime
        description: "Timestamp when translation was created"
        validation: "Required, auto-generated"

    request_count: int
        description: "Number of times this translation has been requested"
        validation: "Required, default 1"
```

## Frontend State Models

### TranslationState
**Purpose**: Frontend state management for translation functionality

```typescript
interface TranslationState {
  isTranslating: boolean
    description: "Whether a translation is currently in progress"

  isTranslated: boolean
    description: "Whether the content is currently in translated state"

  originalContent: string
    description: "The original English content"

  translatedContent: string
    description: "The translated Urdu content (empty if not translated)"

  error: string | null
    description: "Error message if translation failed"

  translationProgress: number
    description: "Progress percentage for translation (0-100)"
}
```