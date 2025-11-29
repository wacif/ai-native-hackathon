"""
Qdrant service for vector database operations.
"""

from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct, Filter
from qdrant_client.http.exceptions import UnexpectedResponse
from ..config.qdrant import qdrant_client, QDRANT_URL, QDRANT_API_KEY
from ..utils.logger import logger
from ..utils.errors import ExternalServiceException

# Default collection name
DEFAULT_COLLECTION = "book_content"


class QdrantService:
    """Service for interacting with Qdrant vector database."""
    
    def __init__(self, collection_name: str = DEFAULT_COLLECTION):
        """
        Initialize Qdrant service.
        
        Args:
            collection_name: Name of the Qdrant collection to use
        """
        self.collection_name = collection_name
        self.client = qdrant_client
    
    def create_collection(self, vector_size: int = 384, distance: Distance = Distance.COSINE) -> bool:
        """
        Create a collection if it doesn't exist.
        
        Args:
            vector_size: Dimension of the vectors (default 384 for BAAI/bge-small-en-v1.5)
            distance: Distance metric to use
            
        Returns:
            bool: True if collection was created or already exists
            
        Raises:
            ExternalServiceException: If collection creation fails
        """
        try:
            # Check if collection exists
            collections = self.client.get_collections().collections
            if any(c.name == self.collection_name for c in collections):
                logger.info(f"Collection '{self.collection_name}' already exists")
                return True
            
            # Create collection
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(size=vector_size, distance=distance)
            )
            logger.info(f"Created collection '{self.collection_name}' successfully")
            return True
            
        except Exception as e:
            logger.error(f"Failed to create collection: {str(e)}")
            raise ExternalServiceException(
                message=f"Failed to create collection '{self.collection_name}'",
                service_name="Qdrant",
                details={"error": str(e)}
            )
    
    def upsert_points(self, points: List[PointStruct]) -> bool:
        """
        Insert or update points in the collection.
        
        Args:
            points: List of PointStruct objects to upsert
            
        Returns:
            bool: True if upsert was successful
            
        Raises:
            ExternalServiceException: If upsert fails
        """
        try:
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )
            logger.info(f"Upserted {len(points)} points to '{self.collection_name}'")
            return True
            
        except Exception as e:
            logger.error(f"Failed to upsert points: {str(e)}")
            raise ExternalServiceException(
                message="Failed to upsert points to Qdrant",
                service_name="Qdrant",
                details={"error": str(e), "num_points": len(points)}
            )
    
    def search(
        self,
        query_vector: List[float],
        limit: int = 5,
        score_threshold: Optional[float] = None,
        filter_conditions: Optional[Filter] = None
    ) -> List[Dict[str, Any]]:
        """
        Search for similar vectors in the collection.
        
        Args:
            query_vector: Query vector to search for
            limit: Maximum number of results to return
            score_threshold: Minimum score threshold for results
            filter_conditions: Optional filter conditions
            
        Returns:
            List of search results with payload and score
            
        Raises:
            ExternalServiceException: If search fails
        """
        try:
            search_results = self.client.query_points(
                collection_name=self.collection_name,
                query=query_vector,
                limit=limit,
                score_threshold=score_threshold,
                query_filter=filter_conditions
            )
            
            results = []
            for point in search_results.points:
                results.append({
                    "id": str(point.id),
                    "score": point.score,
                    "payload": point.payload
                })
            
            logger.debug(f"Search returned {len(results)} results")
            return results
            
        except Exception as e:
            logger.error(f"Failed to search in Qdrant: {str(e)}")
            raise ExternalServiceException(
                message="Failed to search in Qdrant",
                service_name="Qdrant",
                details={"error": str(e)}
            )
    
    def delete_collection(self) -> bool:
        """
        Delete the collection.
        
        Returns:
            bool: True if deletion was successful
            
        Raises:
            ExternalServiceException: If deletion fails
        """
        try:
            self.client.delete_collection(collection_name=self.collection_name)
            logger.info(f"Deleted collection '{self.collection_name}'")
            return True
            
        except Exception as e:
            logger.error(f"Failed to delete collection: {str(e)}")
            raise ExternalServiceException(
                message=f"Failed to delete collection '{self.collection_name}'",
                service_name="Qdrant",
                details={"error": str(e)}
            )
    
    def get_collection_info(self) -> Dict[str, Any]:
        """
        Get information about the collection.
        
        Returns:
            Dictionary with collection information
            
        Raises:
            ExternalServiceException: If fetching info fails
        """
        try:
            info = self.client.get_collection(collection_name=self.collection_name)
            return {
                "name": self.collection_name,
                "vectors_count": info.vectors_count,
                "points_count": info.points_count,
                "status": info.status
            }
            
        except Exception as e:
            logger.error(f"Failed to get collection info: {str(e)}")
            raise ExternalServiceException(
                message=f"Failed to get info for collection '{self.collection_name}'",
                service_name="Qdrant",
                details={"error": str(e)}
            )


# Create default service instance
qdrant_service = QdrantService()

