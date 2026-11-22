"""Thin wrapper around the Gemini API (pip install google-genai).

The key is read from the GEMINI_API_KEY environment variable, or from the
`api_key` argument / ROS parameter. Never commit a real key.
"""
import os

API_KEY_ENV = 'GEMINI_API_KEY'
API_KEY_PLACEHOLDER = 'YOUR_GEMINI_API_KEY_HERE'


class LLMClient:
    """chat(messages) -> text, where messages is a list of (role, text), role 'user' or 'model'."""

    def __init__(self, model, api_key='', temperature=0.0, timeout_s=60.0):
        key = os.environ.get(API_KEY_ENV) or api_key
        if not key or key == API_KEY_PLACEHOLDER:
            raise RuntimeError(
                f'No Gemini API key. Run  export {API_KEY_ENV}=<your key>  '
                f'or set the api_key parameter in config/smart_llm_planner.yaml.')
        from google import genai
        from google.genai import types
        self._types = types
        self._client = genai.Client(
            api_key=key, http_options=types.HttpOptions(timeout=int(timeout_s * 1000)))
        self.model = model
        self.temperature = temperature

    def chat(self, messages, system=None, json_output=False):
        contents = [{'role': role, 'parts': [{'text': text}]} for role, text in messages]
        config = self._types.GenerateContentConfig(
            system_instruction=system,
            temperature=self.temperature,
            response_mime_type='application/json' if json_output else 'text/plain',
        )
        response = self._client.models.generate_content(
            model=self.model, contents=contents, config=config)
        return response.text or ''
