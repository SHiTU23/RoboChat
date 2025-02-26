### Use GPT-4o and Azure AI Search for RAG to generate codes for ur5 robot in ROS and python 

from azure.search.documents import SearchClient
from openai import AzureOpenAI  
from azure.core.credentials import AzureKeyCredential
from azure.search.documents.models import VectorizedQuery
import os  
import json






AZURE_OPENAI_ENDPOINT = "https://aihubthesiswes8755517667.openai.azure.com/"
AZURE_CHAT_MODELNAME = "gpt-4o-codeGenerator"
AZURE_EMBEDDING_MODEL = "text-embedding-3-large"

AZURE_SEARCH_SERVICE = "https://thesis-west-aisearch.search.windows.net"
AZURE_SEARCH_ADMIN_KEY = "RyHN2fSd1HrLqlwpfUuTbVYwmTO4W9rR3OxO9FZko6AzSeDsPllI"

index_name = "rag-robotics-docs-index"

credential = AzureKeyCredential(AZURE_SEARCH_ADMIN_KEY)

def get_embeddings_vector(text):

    response = openai_client.embeddings.create(
        input=text,
        model=AZURE_EMBEDDING_MODEL,
        dimensions=1024,
    )

    embedding = response.data[0].embedding

    return embedding


endpoint = os.getenv("ENDPOINT_URL", "https://aihubthesiswes8755517667.openai.azure.com/")  
deployment = os.getenv("DEPLOYMENT_NAME", "gpt-4o-codeGenerator")  
subscription_key = os.getenv("AZURE_OPENAI_API_KEY", "16e5XT0Seh7kF6knUDWkQsLodd3otgpNR3uJuCOkyXJlVK9181MAJQQJ99BBACfhMk5XJ3w3AAAAACOGguO1")  
# token_provider = get_bearer_token_provider(credential, "https://cognitiveservices.azure.com/.default")
openai_client = AzureOpenAI(
     api_version="2024-06-01",
     azure_endpoint=AZURE_OPENAI_ENDPOINT,
    #  azure_ad_token_provider=token_provider
 )

search_client = SearchClient(
     endpoint=AZURE_SEARCH_SERVICE,
     index_name=index_name,
     credential=credential
 )

# Provide instructions to the model
GROUNDED_PROMPT="""
You are an AI assistant that helps users learn from the information found in the source material.
Answer the query using only the sources provided below.
Use bullets if the answer has multiple points.
If the answer is longer than 3 sentences, provide a summary.
Answer ONLY with the facts listed in the list of sources below. Cite your source when you answer the question
If there isn't enough information below, say you don't know.
Do not generate answers that don't use the sources below.
Query: {query}
\n
Sources:\n{sources}
"""

# Provide the search query. 
# It's hybrid: a keyword search on "query", with text-to-vector conversion for "vector_query".
# The vector query finds 50 nearest neighbor matches in the search index
query="do I need to attach the object the robot wants to pick to the gripper?"
embedding = get_embeddings_vector(query)

vector_query = VectorizedQuery(vector=embedding, k_nearest_neighbors=3, fields="text_vector")

# Set up the search results and the chat thread.
# Retrieve the selected fields from the search index related to the question.
# Search results are limited to the top 5 matches. Limiting top can help you stay under LLM quotas.
search_results = search_client.search(
    search_text=query,
    vector_queries= [vector_query],
    select=["title", "chunk"],
    top=5,
)

# Newlines could be in the OCR'd content or in PDFs, as is the case for the sample PDFs used for this tutorial.
# Use a unique separator to make the sources distinct. 
# We chose repeated equal signs (=) followed by a newline because it's unlikely the source documents contain this sequence.
sources_formatted = "=================\n".join([f'TITLE: {document["title"]}, CONTENT: {document["chunk"]}' for document in search_results])

response = openai_client.chat.completions.create(
    messages=[
        {
            "role": "user",
            "content": GROUNDED_PROMPT.format(query=query, sources=sources_formatted)
        }
    ],
    model=AZURE_CHAT_MODELNAME
)

print(response.choices[0].message.content)