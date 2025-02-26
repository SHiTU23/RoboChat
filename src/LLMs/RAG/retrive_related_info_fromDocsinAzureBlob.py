from azure.search.documents import SearchClient
from azure.search.documents.models import VectorizableTextQuery
from azure.core.credentials import AzureKeyCredential


AZURE_SEARCH_SERVICE = "https://thesis-west-aisearch.search.windows.net"
AZURE_SEARCH_ADMIN_KEY = "RyHN2fSd1HrLqlwpfUuTbVYwmTO4W9rR3OxO9FZko6AzSeDsPllI"

index_name = "rag-robotics-docs-index"
credential = AzureKeyCredential(AZURE_SEARCH_ADMIN_KEY)

# Vector Search using text-to-vector conversion of the querystring
query = "how to pick an object?"  

search_client = SearchClient(endpoint=AZURE_SEARCH_SERVICE, credential=credential, index_name=index_name)
vector_query = VectorizableTextQuery(text=query, k_nearest_neighbors=50, fields="text_vector")
  
results = search_client.search(  
    search_text=query,  
    vector_queries= [vector_query],
    select=["chunk"],
    top=1
)  
  
for result in results:  
    print(f"Score: {result['@search.score']}")
    print(f"Chunk: {result['chunk']}")