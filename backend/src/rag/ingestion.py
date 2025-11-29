import os
from pathlib import Path
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
from dotenv import load_dotenv

load_dotenv()

QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_COLLECTION_NAME = "book_content"

# Initialize Qdrant client with FastEmbed
client = QdrantClient(
    url=QDRANT_URL,
    api_key=QDRANT_API_KEY,
    prefer_grpc=False,
)

def chunk_text(text: str, chunk_size: int = 500, overlap: int = 50) -> list[str]:
    """Split text into overlapping chunks."""
    words = text.split()
    chunks = []

    for i in range(0, len(words), chunk_size - overlap):
        chunk = ' '.join(words[i:i + chunk_size])
        if chunk.strip():
            chunks.append(chunk)

    return chunks

def process_markdown_file(filepath: str) -> str:
    """Read and return markdown file content."""
    with open(filepath, 'r', encoding='utf-8') as f:
        return f.read()

def extract_metadata_from_path(relative_path: str, doc_type: str) -> dict:
    """
    Extract chapter_id, module_id, and page_url from file path
    
    Args:
        relative_path: Relative path from base (e.g., "docs/physical-ai/chapter1.md")
        doc_type: Type of document ("docs" or "blog")
    
    Returns:
        Dictionary with metadata fields
    """
    path = Path(relative_path)
    parts = path.parts
    
    metadata = {
        "chapter_id": None,
        "module_id": None,
        "page_url": None
    }
    
    if doc_type == "docs" and len(parts) >= 2:
        # Extract module_id (e.g., "physical-ai" from "docs/physical-ai/chapter1.md")
        if len(parts) >= 3:
            metadata["module_id"] = parts[1]
            # Extract chapter_id from filename (without extension)
            metadata["chapter_id"] = path.stem
            # Construct page URL
            metadata["page_url"] = f"/docs/{parts[1]}/{path.stem}"
        elif len(parts) == 2:
            # Top-level docs file
            metadata["chapter_id"] = path.stem
            metadata["page_url"] = f"/docs/{path.stem}"
    elif doc_type == "blog":
        metadata["chapter_id"] = path.stem
        metadata["page_url"] = f"/blog/{path.stem}"
    
    return metadata

def collect_markdown_files(base_path: str) -> list[dict]:
    """Collect all markdown files from docs and blog directories."""
    documents = []
    base = Path(base_path)

    # Collect from docs
    docs_path = base / "docs"
    if docs_path.exists():
        for md_file in docs_path.rglob("*.md"):
            content = process_markdown_file(str(md_file))
            relative_path = str(md_file.relative_to(base))
            metadata = extract_metadata_from_path(relative_path, "docs")
            
            documents.append({
                "content": content,
                "source": relative_path,
                "type": "docs",
                **metadata
            })
        for mdx_file in docs_path.rglob("*.mdx"):
            content = process_markdown_file(str(mdx_file))
            relative_path = str(mdx_file.relative_to(base))
            metadata = extract_metadata_from_path(relative_path, "docs")
            
            documents.append({
                "content": content,
                "source": relative_path,
                "type": "docs",
                **metadata
            })

    # Collect from blog
    blog_path = base / "blog"
    if blog_path.exists():
        for md_file in blog_path.rglob("*.md"):
            content = process_markdown_file(str(md_file))
            relative_path = str(md_file.relative_to(base))
            metadata = extract_metadata_from_path(relative_path, "blog")
            
            documents.append({
                "content": content,
                "source": relative_path,
                "type": "blog",
                **metadata
            })
        for mdx_file in blog_path.rglob("*.mdx"):
            content = process_markdown_file(str(mdx_file))
            relative_path = str(mdx_file.relative_to(base))
            metadata = extract_metadata_from_path(relative_path, "blog")
            
            documents.append({
                "content": content,
                "source": relative_path,
                "type": "blog",
                **metadata
            })

    return documents

def ingest_data(book_path: str = ".."):
    """Ingest markdown files into Qdrant using FastEmbed."""
    print("Starting data ingestion...")

    # Collect all markdown files
    documents = collect_markdown_files(book_path)
    print(f"Found {len(documents)} markdown files")

    if not documents:
        print("No documents found to ingest!")
        return

    # Create chunks from documents
    all_chunks = []
    for doc in documents:
        chunks = chunk_text(doc["content"])
        for i, chunk in enumerate(chunks):
            all_chunks.append({
                "text": chunk,
                "source": doc["source"],
                "type": doc["type"],
                "chunk_id": i,
                "chapter_id": doc.get("chapter_id"),
                "module_id": doc.get("module_id"),
                "page_url": doc.get("page_url")
            })

    print(f"Created {len(all_chunks)} chunks from documents")

    # Check if collection exists, if so delete it
    try:
        client.delete_collection(collection_name=QDRANT_COLLECTION_NAME)
        print(f"Deleted existing collection '{QDRANT_COLLECTION_NAME}'")
    except Exception:
        pass

    # Create collection with FastEmbed support
    # FastEmbed uses 384-dimensional vectors by default (all-MiniLM-L6-v2)
    from qdrant_client.models import PayloadSchemaType
    
    client.create_collection(
        collection_name=QDRANT_COLLECTION_NAME,
        vectors_config=VectorParams(
            size=384,
            distance=Distance.COSINE,
        ),
    )
    print(f"Created collection '{QDRANT_COLLECTION_NAME}'")
    
    # Create payload indexes for filtering
    client.create_payload_index(
        collection_name=QDRANT_COLLECTION_NAME,
        field_name="chapter_id",
        field_schema=PayloadSchemaType.KEYWORD
    )
    print("Created index for 'chapter_id'")
    
    client.create_payload_index(
        collection_name=QDRANT_COLLECTION_NAME,
        field_name="module_id",
        field_schema=PayloadSchemaType.KEYWORD
    )
    print("Created index for 'module_id'")

    # Initialize FastEmbed for generating embeddings
    from fastembed import TextEmbedding
    embedding_model = TextEmbedding(model_name="BAAI/bge-small-en-v1.5")

    # Generate embeddings and create points
    points = []
    print("Generating embeddings...")

    for idx, chunk in enumerate(all_chunks):
        # Generate embedding for the chunk
        embedding = list(embedding_model.embed([chunk["text"]]))[0]

        # Create point
        points.append(
            PointStruct(
                id=idx,
                vector=embedding.tolist(),
                payload={
                    "text": chunk["text"],
                    "source": chunk["source"],
                    "type": chunk["type"],
                    "chunk_id": chunk["chunk_id"],
                    "chapter_id": chunk.get("chapter_id"),
                    "module_id": chunk.get("module_id"),
                    "page_url": chunk.get("page_url")
                }
            )
        )

    # Upload points to Qdrant
    print(f"Uploading {len(points)} points to Qdrant...")
    client.upsert(
        collection_name=QDRANT_COLLECTION_NAME,
        points=points
    )

    print(f"✅ Successfully ingested {len(all_chunks)} chunks into Qdrant collection '{QDRANT_COLLECTION_NAME}'")

if __name__ == "__main__":
    ingest_data()