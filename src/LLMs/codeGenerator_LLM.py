### Use GPT-4o and Azure AI Search for RAG to generate codes for ur5 robot in ROS and python 

from azure.search.documents import SearchClient
from openai import AzureOpenAI  
from azure.core.credentials import AzureKeyCredential
from azure.search.documents.models import VectorizedQuery
import os  
import json



class second_layer_LLM:
    def __init__(self):
        self._AZURE_OPENAI_ENDPOINT = "https://aihubthesiswes8755517667.openai.azure.com/"
        self._AZURE_CHAT_MODELNAME = "gpt-4o-codeGenerator"
        self._AZURE_EMBEDDING_MODEL = "text-embedding-3-large"

        self._AZURE_SEARCH_SERVICE = "https://thesis-west-aisearch.search.windows.net"
        self._AZURE_SEARCH_ADMIN_KEY = "RyHN2fSd1HrLqlwpfUuTbVYwmTO4W9rR3OxO9FZko6AzSeDsPllI"

        _index_name = "docs-index"

        _credential = AzureKeyCredential(self._AZURE_SEARCH_ADMIN_KEY)
        self._openai_client = AzureOpenAI(
                                            api_version="2024-06-01",
                                            azure_endpoint=self._AZURE_OPENAI_ENDPOINT,
                                         )

        self._search_client = SearchClient(
                                            endpoint=self._AZURE_SEARCH_SERVICE,
                                            index_name=_index_name,
                                            credential=_credential
                                          )

        self._chat_history = []
    def get_embeddings_vector(self, text):
        response = self._openai_client.embeddings.create(
                                                            input=text,
                                                            model=self._AZURE_EMBEDDING_MODEL,
                                                            dimensions=1024,
                                                        )
        embedding = response.data[0].embedding
        return embedding

    def generate_answer(self, query, RAG_inUse=True):
        if RAG_inUse:
            # Provide instructions to the model
            _GROUNDED_PROMPT="""
            You are an AI assistant for a UR5 robot in ROS and Gezebo. Your task is to generate action plans and then Python codes for them to perform the robotics task requested in the query when the query is a robotic task, otherwise, provide a brief (2-3 sentence) reliable answer to the quetions that are not a robotic_task.
            
            - When generating a code, provide a step-by-step plan for the task.
            - Get help from the provided resourcesfor answering to the query and make your final response more correct. 
            - Do not use methods of the class from the provided resources in the answer directly, instead use the necessary parts in it for performimng the whole task.
            - Refer to the chat history for context if needed.
            - Use bullet points for multi-point answers.
            - If you don't know the answer, say you don't have enough information.
            - references the sources when using them.
            
            At the end, include a concise summary labeled **"history:"** for future reference.
            
            Query: {query}
            \n
            ++++++++++++++++
            Chat_history: {history}   
            \n
            Do not change the format of the references below.
            Sources:\n{sources}
            """

            # Provide the search query. 
            # The vector query finds 3 nearest neighbor matches in the search index
            # query="pick the green cube."
            _embedding = self.get_embeddings_vector(query)
            _vector_query = VectorizedQuery(vector=_embedding, k_nearest_neighbors=3, fields="text_vector")

            # Set up the search results and the chat thread.
            # Retrieve the selected fields from the search index related to the question.
            # Search results are limited to the top 5 matches. Limiting top can help you stay under LLM quotas.
            search_results = self._search_client.search(
                                                        search_text=query,
                                                        vector_queries= [_vector_query],
                                                        select=["title", "chunk"],
                                                        top=5,
                                                        )

            # Use a unique separator to make the sources distinct, such as === 
            _sources_formatted = "=================\n".join([f'TITLE: {document["title"]}, CONTENT: {document["chunk"]}' for document in search_results])
            _content = _GROUNDED_PROMPT.format(query=query, history=self._chat_history, sources=_sources_formatted)

        else:
            _GROUNDED_PROMPT = """
            You are an AI assistant for a UR5 robot in ROS and Gezebo that generates action plans and then Python codes for it to perform the robotics task requested in the query when the query is a robotic task. otherwise, provide 2-3 sentences short reliable answers to the quetions that are not a robotic_task.
            Use bullet points if the answer has multiple points.
            If you don't know the answer, say you don't have enough information.
            At the end give a very short summary to be used as a history in the chatm label "history:".
            Query: {query}
            """
            _content = _GROUNDED_PROMPT.format(query=query)

        response = self._openai_client.chat.completions.create(
                                                                messages=[
                                                                    {
                                                                        "role": "user",
                                                                        "content": _content
                                                                    }
                                                                ],
                                                                model=self._AZURE_CHAT_MODELNAME,
                                                                max_tokens=5000,  
                                                                temperature=0.5,  
                                                                top_p=0.95,  
                                                                frequency_penalty=0,  
                                                                presence_penalty=0,
                                                                stop=None,  
                                                                stream=False
                                                            )
        llm_response = response.choices[0].message.content
        summary = self.extract_summary(llm_response)
        history = {
                    "query": query,
                    "summary": summary
                 }
        self._chat_history.append(history)


        return llm_response
    
    def extract_summary(self, text):
        keyword = "history"
        index = text.lower().find(keyword) 
        
        if index != -1:  # If "summary" is found
            return text[index:] 
        else:
            return None 

if __name__ == "__main__":
    llm = second_layer_LLM()
    while True:
        query = input("Enter query: ")
        response = llm.generate_answer(query, RAG_inUse=True)
        print(response)
        print("=====================================")