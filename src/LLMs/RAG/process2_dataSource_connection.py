### connecting to Blob storage

from azure.search.documents.indexes import SearchIndexerClient
from azure.search.documents.indexes.models import (
    SearchIndexerDataContainer,
    SearchIndexerDataSourceConnection
)
from azure.core.credentials import AzureKeyCredential
from openai import AzureOpenAI

'''
AZURE_OPENAI_ENDPOINT = "https://aihubthesiswes8755517667.openai.azure.com/"
AZURE_OPENAI_API_KEY = "16e5XT0Seh7kF6knUDWkQsLodd3otgpNR3uJuCOkyXJlVK9181MAJQQJ99BBACfhMk5XJ3w3AAAAACOGguO1"
AZURE_CHAT_MODELNAME = "gpt-4o-codeGenerator"

AZURE_EMBEDDING_MODEL = "text-embedding-3-large"

AZURE_SEARCH_SERVICE = "https://thesis-west-aisearch.search.windows.net"
AZURE_SEARCH_ADMIN_KEY = "RyHN2fSd1HrLqlwpfUuTbVYwmTO4W9rR3OxO9FZko6AzSeDsPllI"
### for the connection string, go to Azure portal, storage account, under datastorage go to containers, choose your container name and go to Shared access token under Setting menu, and gnerate one and pick the URL 
AZURE_STORAGE_CONNECTION_STRING = "ContainerSharedAccessUri=https://aihubthesiswes4047422348.blob.core.windows.net/robotics-data?sp=rl&st=2025-02-26T09:48:15Z&se=2025-06-01T17:48:15Z&spr=https&sv=2022-11-02&sr=c&sig=1ugZhqeGwuwMrH89Bq0SqnYwS0QMzH1tyHEpqvtJDTM%3D"
credential = AzureKeyCredential(AZURE_SEARCH_ADMIN_KEY)
datasource_name = "rag-robotics-docs-datasource-update"
'''

def connect_dataSource(azure_search_service, azure_storage_connection_string, credential, datasource_name, blob_storage_name):
    # Create a data source 
    indexer_client = SearchIndexerClient(
                                        endpoint=azure_search_service, 
                                        credential=credential)
    container = SearchIndexerDataContainer(name=blob_storage_name)
    data_source_connection = SearchIndexerDataSourceConnection(
        name=datasource_name,
        type="azureblob",
        connection_string=azure_storage_connection_string,
        container=container
    )
    data_source = indexer_client.create_or_update_data_source_connection(data_source_connection)

    print(f"Data source '{data_source.name}' created or updated")