### connecting to Blob storage

from azure.search.documents.indexes import SearchIndexerClient
from azure.search.documents.indexes.models import (
    SearchIndexerDataContainer,
    SearchIndexerDataSourceConnection
)
from azure.core.credentials import AzureKeyCredential
from openai import AzureOpenAI


AZURE_OPENAI_ENDPOINT = "https://aihubthesiswes8755517667.openai.azure.com/"
AZURE_OPENAI_API_KEY = "16e5XT0Seh7kF6knUDWkQsLodd3otgpNR3uJuCOkyXJlVK9181MAJQQJ99BBACfhMk5XJ3w3AAAAACOGguO1"
AZURE_CHAT_MODELNAME = "gpt-4o-codeGenerator"

AZURE_EMBEDDING_MODEL = "text-embedding-3-large"

AZURE_SEARCH_SERVICE = "https://thesis-west-aisearch.search.windows.net"
AZURE_SEARCH_ADMIN_KEY = "RyHN2fSd1HrLqlwpfUuTbVYwmTO4W9rR3OxO9FZko6AzSeDsPllI"
AZURE_STORAGE_CONNECTION_STRING = "ContainerSharedAccessUri=https://aihubthesiswes4047422348.blob.core.windows.net/robotics-data?sp=rl&st=2025-02-25T12:52:05Z&se=2025-02-25T20:52:05Z&spr=https&sv=2022-11-02&sr=c&sig=2uU3Mo%2FHX4uatg44VoAaOi3SCrpRhfOSQW%2Fbdlk2PX8%3D"

openai_client = AzureOpenAI(
    azure_endpoint=AZURE_OPENAI_ENDPOINT,
    api_key=AZURE_OPENAI_API_KEY,
    api_version="2024-06-01"
)

datasource_name = "rag-robotics-chat-datasource"
credential = AzureKeyCredential(AZURE_SEARCH_ADMIN_KEY)

# Create a data source 
indexer_client = SearchIndexerClient(
                                     endpoint=AZURE_SEARCH_SERVICE, 
                                     credential=credential)
container = SearchIndexerDataContainer(name="robotics-data")
data_source_connection = SearchIndexerDataSourceConnection(
    name=datasource_name,
    type="azureblob",
    connection_string=AZURE_STORAGE_CONNECTION_STRING,
    container=container
)
data_source = indexer_client.create_or_update_data_source_connection(data_source_connection)

print(f"Data source '{data_source.name}' created or updated")