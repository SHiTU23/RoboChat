from process1_create_index_schema import create_index_schema
from process2_dataSource_connection import connect_dataSource
from process3_create_skillset import create_skillset
from process4_create_indexer import create_indexer
from azure.core.credentials import AzureKeyCredential
from azure.storage.blob import ContainerClient

AZURE_OPENAI_ENDPOINT = "https://aihubthesiswes8755517667.openai.azure.com/"
AZURE_OPENAI_API_KEY = "16e5XT0Seh7kF6knUDWkQsLodd3otgpNR3uJuCOkyXJlVK9181MAJQQJ99BBACfhMk5XJ3w3AAAAACOGguO1"
AZURE_CHAT_MODELNAME = "gpt-4o-codeGenerator"

AZURE_EMBEDDING_MODEL = "text-embedding-3-large"

AZURE_SEARCH_SERVICE = "https://thesis-west-aisearch.search.windows.net"
AZURE_SEARCH_ADMIN_KEY = "RyHN2fSd1HrLqlwpfUuTbVYwmTO4W9rR3OxO9FZko6AzSeDsPllI"
### for the connection string, go to Azure portal, storage account, under datastorage go to containers, choose your container name and go to Shared access token under Setting menu, and gnerate one and pick the URL 
AZURE_BLOB_SAS_URL = "ContainerSharedAccessUri=https://aihubthesiswes4047422348.blob.core.windows.net/robotics-data?sp=racwl&st=2025-03-02T10:08:14Z&se=2025-06-01T18:08:14Z&spr=https&sv=2022-11-02&sr=c&sig=ldcqMmhVNdjePhmTf0g4Ti8pth3Y3A%2FK0hMQgSCIxe0%3D"
credential = AzureKeyCredential(AZURE_SEARCH_ADMIN_KEY)


### final names for the index, datasource, skillset and indexer
index_name = "docs-index"
datasource_name = "docs-ds"
CONTAINER_NAME = "robotics-data"
skillset_name = "docs-ss"
indexer_name = "docs-idxr" 


## First step: create the schema for the index
create_index_schema(AZURE_OPENAI_ENDPOINT, AZURE_SEARCH_SERVICE, credential, index_name)

### Second step: connect the datasource (Blob storage)
connect_dataSource(AZURE_SEARCH_SERVICE, AZURE_BLOB_SAS_URL, credential, datasource_name, CONTAINER_NAME)

### Third step: create the skillset
create_skillset(AZURE_OPENAI_ENDPOINT, AZURE_SEARCH_SERVICE, credential, index_name, skillset_name)

### Fourth step: create the indexer
create_indexer(AZURE_SEARCH_SERVICE, credential, index_name, datasource_name, skillset_name, indexer_name)


