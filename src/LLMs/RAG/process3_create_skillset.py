from azure.search.documents.indexes.models import (
    SplitSkill,
    InputFieldMappingEntry,
    OutputFieldMappingEntry,
    AzureOpenAIEmbeddingSkill,
    EntityRecognitionSkill,
    SearchIndexerIndexProjection,
    SearchIndexerIndexProjectionSelector,
    SearchIndexerIndexProjectionsParameters,
    IndexProjectionMode,
    SearchIndexerSkillset,
    CognitiveServicesAccountKey
)
from azure.search.documents.indexes import SearchIndexerClient
from azure.core.credentials import AzureKeyCredential

'''
# Azure Configuration
azure_openai_endpoint = "https://aihubthesiswes8755517667.openai.azure.com/"
AZURE_OPENAI_API_KEY = "16e5XT0Seh7kF6knUDWkQsLodd3otgpNR3uJuCOkyXJlVK9181MAJQQJ99BBACfhMk5XJ3w3AAAAACOGguO1"
AZURE_CHAT_MODELNAME = "gpt-4o-codeGenerator"

AZURE_EMBEDDING_MODEL = "text-embedding-3-large"
EMBEDDING_DIMENSION = 3072

AZURE_SEARCH_SERVICE = "https://thesis-west-aisearch.search.windows.net"
AZURE_SEARCH_ADMIN_KEY = "RyHN2fSd1HrLqlwpfUuTbVYwmTO4W9rR3OxO9FZko6AzSeDsPllI"

AZURE_STORAGE_CONNECTION_STRING = "ContainerSharedAccessUri=https://aihubthesiswes4047422348.blob.core.windows.net/robotics-data?sp=rl&st=2025-02-25T12:52:05Z&se=2025-02-25T20:52:05Z&spr=https&sv=2022-11-02&sr=c&sig=2uU3Mo%2FHX4uatg44VoAaOi3SCrpRhfOSQW%2Fbdlk2PX8%3D"

credential = AzureKeyCredential(AZURE_SEARCH_ADMIN_KEY)

index_name = "rag-robotics-docs-index-update"
skillset_name = "rag-robotics-docs-ss-update"
'''


def create_skillset(azure_openai_endpoint, azure_search_service, credential, index_name, skillset_name):
    # Create a skillset  
    split_skill = SplitSkill(  
        description="Split skill to chunk documents",  
        text_split_mode="pages",  
        context="/document",  
        maximum_page_length=2000,  
        page_overlap_length=500,  
        inputs=[  
            InputFieldMappingEntry(name="text", source="/document/content"),  
        ],  
        outputs=[  
            OutputFieldMappingEntry(name="textItems", target_name="pages")  
        ],  
    )  
    
    embedding_skill = AzureOpenAIEmbeddingSkill(  
        description="Skill to generate embeddings via Azure OpenAI",  
        context="/document/pages/*",  
        resource_url=azure_openai_endpoint,  
        deployment_name="text-embedding-3-large",  
        model_name="text-embedding-3-large",
        dimensions=1024,
        inputs=[  
            InputFieldMappingEntry(name="text", source="/document/pages/*"),  
        ],  
        outputs=[  
            OutputFieldMappingEntry(name="embedding", target_name="text_vector")  
        ],  
    )


    index_projections = SearchIndexerIndexProjection(  
        selectors=[  
            SearchIndexerIndexProjectionSelector(  
                target_index_name=index_name,  
                parent_key_field_name="parent_id",  
                source_context="/document/pages/*",  
                mappings=[  
                    InputFieldMappingEntry(name="chunk", source="/document/pages/*"),  
                    InputFieldMappingEntry(name="text_vector", source="/document/pages/*/text_vector"),
                    InputFieldMappingEntry(name="title", source="/document/metadata_storage_name"),  
                ],  
            ),  
        ],  
        parameters=SearchIndexerIndexProjectionsParameters(  
            projection_mode=IndexProjectionMode.SKIP_INDEXING_PARENT_DOCUMENTS  
        ),  
    ) 

    # cognitive_services_account = CognitiveServicesAccountKey(key=AZURE_STORAGE_CONNECTION_STRING)

    skills = [split_skill, embedding_skill]

    skillset = SearchIndexerSkillset(  
        name=skillset_name,  
        description="Skillset to chunk documents and generating embeddings",  
        skills=skills,  
        index_projection=index_projections,
        # cognitive_services_account=cognitive_services_account
    )
    
    client = SearchIndexerClient(endpoint=azure_search_service, credential=credential)  
    client.create_or_update_skillset(skillset)  
    print(f"{skillset.name} created")