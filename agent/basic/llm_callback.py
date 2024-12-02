from langchain_core.callbacks import BaseCallbackHandler
from typing import TYPE_CHECKING, Any, Optional, TypeVar, Union
from uuid import UUID
from langchain_core.outputs import LLMResult
import re

def decode_unicode_escapes(text):
    def replace_unicode(match):
        return bytes(match.group(0), 'utf-8').decode('unicode_escape')
    
    # 只替换形如 \u1234 的 Unicode 转义序列
    return re.sub(r'\\u[0-9a-fA-F]{4}', replace_unicode, text)

class MyCustomHandler(BaseCallbackHandler):
    def on_llm_start(
        self,
        serialized: dict[str, Any],
        prompts: list[str],
        *,
        run_id: UUID,
        parent_run_id: Optional[UUID] = None,
        tags: Optional[list[str]] = None,
        metadata: Optional[dict[str, Any]] = None,
        **kwargs: Any,
    ) -> Any:
        """Run when LLM starts running.

        **ATTENTION**: This method is called for non-chat models (regular LLMs). If
            you're implementing a handler for a chat model,
            you should use on_chat_model_start instead.

        Args:
            serialized (Dict[str, Any]): The serialized LLM.
            prompts (List[str]): The prompts.
            run_id (UUID): The run ID. This is the ID of the current run.
            parent_run_id (UUID): The parent run ID. This is the ID of the parent run.
            tags (Optional[List[str]]): The tags.
            metadata (Optional[Dict[str, Any]]): The metadata.
            kwargs (Any): Additional keyword arguments.
        """
        print_message = prompts[0].replace('\\n', '\n')
        print_message = decode_unicode_escapes(print_message)

        print(f"""\n\n\n\n\n\n\nprompts:\n{print_message}
                \n\n\n\n\n\n\n
              """)
        
    def on_llm_end(
        self,
        response: LLMResult,
        *,
        run_id: UUID,
        parent_run_id: Optional[UUID] = None,
        **kwargs: Any,
    ) -> Any:
        """Run when LLM ends running.

        Args:
            response (LLMResult): The response which was generated.
            run_id (UUID): The run ID. This is the ID of the current run.
            parent_run_id (UUID): The parent run ID. This is the ID of the parent run.
            kwargs (Any): Additional keyword arguments.
        """
        response_text = response.generations[0][0].text
        processed_text = decode_unicode_escapes(response_text)
        print(f"""\n\n\n\n\n\n\n
                response_text:\n
                {processed_text}
                    \n\n\n\n\n\n\n
                """)