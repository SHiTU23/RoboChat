from azure.search.documents.indexes.models import (
    SearchIndexer,
    FieldMapping
)
from azure.search.documents.indexes import SearchIndexerClient
from azure.core.credentials import AzureKeyCredential

# Azure Configuration
AZURE_OPENAI_ENDPOINT = "https://aihubthesiswes8755517667.openai.azure.com/"
AZURE_OPENAI_API_KEY = "16e5XT0Seh7kF6knUDWkQsLodd3otgpNR3uJuCOkyXJlVK9181MAJQQJ99BBACfhMk5XJ3w3AAAAACOGguO1"
AZURE_CHAT_MODELNAME = "gpt-4o-codeGenerator"

AZURE_EMBEDDING_MODEL = "text-embedding-3-large"

AZURE_SEARCH_SERVICE = "https://thesis-west-aisearch.search.windows.net"
AZURE_SEARCH_ADMIN_KEY = "RyHN2fSd1HrLqlwpfUuTbVYwmTO4W9rR3OxO9FZko6AzSeDsPllI"

credential = AzureKeyCredential(AZURE_SEARCH_ADMIN_KEY)

# Create an indexer  
index_name = "rag-robotics-docs-index"
datasource_name = "rag-robotics-docs-datasource"
skillset_name = "rag-robotics-docs-ss"
indexer_name = "rag-robotics-docs-idxr" 

indexer_parameters = None

indexer = SearchIndexer(  
    name=indexer_name,  
    description="Indexer to index documents and generate embeddings",  
    skillset_name=skillset_name,  
    target_index_name=index_name,  
    data_source_name=datasource_name,
    # Map the metadata_storage_name field to the title field in the index to display the PDF title in the search results  
    field_mappings=[FieldMapping(source_field_name="metadata_storage_name", target_field_name="title")],
    parameters=indexer_parameters
)  

# Create and run the indexer  
indexer_client = SearchIndexerClient(endpoint=AZURE_SEARCH_SERVICE, credential=credential)  
indexer_result = indexer_client.create_or_update_indexer(indexer)  

print(f' {indexer_name} is created and running. Give the indexer a few minutes before running a query.')