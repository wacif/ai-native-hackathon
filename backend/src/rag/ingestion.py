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

def collect_markdown_files(base_path: str) -> list[dict]:
    """Collect all markdown files from docs and blog directories."""
    documents = []
    base = Path(base_path)

    # Collect from docs
    docs_path = base / "docs"
    if docs_path.exists():
        for md_file in docs_path.rglob("*.md"):
            content = process_markdown_file(str(md_file))
            documents.append({
                "content": content,
                "source": str(md_file.relative_to(base)),
                "type": "docs"
            })
        for mdx_file in docs_path.rglob("*.mdx"):
            content = process_markdown_file(str(mdx_file))
            documents.append({
                "content": content,
                "source": str(mdx_file.relative_to(base)),
                "type": "docs"
            })

    # Collect from blog
    blog_path = base / "blog"
    if blog_path.exists():
        for md_file in blog_path.rglob("*.md"):
            content = process_markdown_file(str(md_file))
            documents.append({
                "content": content,
                "source": str(md_file.relative_to(base)),
                "type": "blog"
            })
        for mdx_file in blog_path.rglob("*.mdx"):
            content = process_markdown_file(str(mdx_file))
            documents.append({
                "content": content,
                "source": str(mdx_file.relative_to(base)),
                "type": "blog"
            })

    return documents

def ingest_data(book_path: str = "../my-book"):
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
                "chunk_id": i
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
    client.create_collection(
        collection_name=QDRANT_COLLECTION_NAME,
        vectors_config=VectorParams(
            size=384,
            distance=Distance.COSINE,
        ),
    )
    print(f"Created collection '{QDRANT_COLLECTION_NAME}'")

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
                    "chunk_id": chunk["chunk_id"]
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