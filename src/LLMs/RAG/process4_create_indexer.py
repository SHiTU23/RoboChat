from azure.search.documents.indexes.models import (
    SearchIndexer,
    FieldMapping
)
from azure.search.documents.indexes import SearchIndexerClient
from azure.core.credentials import AzureKeyCredential


def create_indexer(azure_search_service, credential, index_name, datasource_name, skillset_name, indexer_name):
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
    indexer_client = SearchIndexerClient(endpoint=azure_search_service, credential=credential)  
    indexer_result = indexer_client.create_or_update_indexer(indexer)  

    print(f'result for {indexer_name} is created and running. Give the indexer a few minutes before running a query.')

if __name__ == '__main__':
    # Azure Configuration
    AZURE_SEARCH_SERVICE = "https://thesis-west-aisearch.search.windows.net"
    AZURE_SEARCH_ADMIN_KEY = "RyHN2fSd1HrLqlwpfUuTbVYwmTO4W9rR3OxO9FZko6AzSeDsPllI"

    credential = AzureKeyCredential(AZURE_SEARCH_ADMIN_KEY)

    # Create an indexer  
    index_name = "rag2-docs-index"
    datasource_name = "rag2-docs-ds"
    blob_storage_name = "robotics-data"
    skillset_name = "rag2-docs-ss"
    indexer_name = "rag2-docs-idxr" 


    create_indexer(AZURE_SEARCH_SERVICE, credential, index_name, datasource_name, skillset_name, indexer_name)