# 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
import openai

# Set your OpenAI API key
openai.api_key = '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'

# Example prompt
prompt = "You are a poetic assistant, skilled in explaining complex programming concepts with creative flair"

# Use the OpenAI API to generate a completion
completion = openai.chat.completions.create(
  model="gpt-3.5-turbo",  # specify the engine (choose one based on your needs)
  messages=[
    {"role": "system", "content": prompt},
    {"role": "user", "content": "Compose a poem that explains the concept of recursion in programming."}
  ]
)

# Print the generated text
print(completion.choices[0].message)
