### the first step is to create the schema for the index
### the schema will include the fields that will be used in the index

from azure.core.credentials import AzureKeyCredential
from azure.search.documents.indexes import SearchIndexClient
from azure.search.documents.indexes.models import (
    SearchIndex,
    SearchField,
    SearchFieldDataType,
    VectorSearch,
    HnswAlgorithmConfiguration,
    VectorSearchProfile,
    SearchFieldDataType,
    VectorSearch,
    HnswAlgorithmConfiguration,
    VectorSearchProfile,
    AzureOpenAIVectorizer,
    AzureOpenAIVectorizerParameters,
    SearchIndex
) 

'''
AZURE_OPENAI_ENDPOINT = "https://aihubthesiswes8755517667.openai.azure.com/"

AZURE_SEARCH_SERVICE = "https://thesis-west-aisearch.search.windows.net"
AZURE_SEARCH_ADMIN_KEY = "RyHN2fSd1HrLqlwpfUuTbVYwmTO4W9rR3OxO9FZko6AzSeDsPllI"
AZURE_STORAGE_CONNECTION_STRING = "ContainerSharedAccessUri=https://aihubthesiswes4047422348.blob.core.windows.net/robotics-data?sp=rl&st=2025-02-25T12:52:05Z&se=2025-02-25T20:52:05Z&spr=https&sv=2022-11-02&sr=c&sig=2uU3Mo%2FHX4uatg44VoAaOi3SCrpRhfOSQW%2Fbdlk2PX8%3D"

index_name = "rag-robotics-docs-index-update"

credential = AzureKeyCredential(AZURE_SEARCH_ADMIN_KEY)
'''

def create_index_schema(azure_openai_endpoint, azure_search_service, credential, index_name):
    ### Create a search index  
    search_index_client = SearchIndexClient(
        endpoint=azure_search_service, 
        index_name=index_name, 
        credential=credential
    )

    fields = [
        SearchField(name="parent_id", type=SearchFieldDataType.String),  
        SearchField(name="title", type=SearchFieldDataType.String),
        SearchField(name="chunk_id", type=SearchFieldDataType.String, key=True, sortable=True, filterable=True, facetable=True, analyzer_name="keyword"),  
        SearchField(name="chunk", type=SearchFieldDataType.String, sortable=False, filterable=False, facetable=False),  
        SearchField(name="text_vector", type=SearchFieldDataType.Collection(SearchFieldDataType.Single), vector_search_dimensions=1024, vector_search_profile_name="myHnswProfile")
        ]  
    
    ### Configure the vector search configuration  
    vector_search = VectorSearch(  
        algorithms=[  
            HnswAlgorithmConfiguration(name="myHnsw"),
        ],  
        profiles=[  
            VectorSearchProfile(  
                name="myHnswProfile",  
                algorithm_configuration_name="myHnsw",
                vectorizer_name="myOpenAI",    
            )
        ],  
        vectorizers=[  
            AzureOpenAIVectorizer(  
                vectorizer_name="myOpenAI",  
                kind="azureOpenAI",  
                parameters=AzureOpenAIVectorizerParameters(  
                    resource_url=azure_openai_endpoint,  
                    deployment_name="text-embedding-3-large",
                    model_name="text-embedding-3-large"
                ),
            ),  
        ], 
    )  
    
    # # Create the search index
    index = SearchIndex(name=index_name, fields=fields, vector_search=vector_search)  
    result = search_index_client.create_or_update_index(index)  
    print(f"{result.name} created")