FROM python:3.10-slim

WORKDIR /app

# Install git (sometimes needed for Hugging Face Spaces)
RUN apt-get update && apt-get install -y git && rm -rf /var/lib/apt/lists/*

# Copy the optimized requirements for Hugging Face Spaces
COPY requirements_hf_spaces.txt .
RUN pip install --no-cache-dir -r requirements_hf_spaces.txt

# Copy the application code
COPY ./src ./src
COPY app.py .

# Expose the port that Hugging Face Spaces expects
EXPOSE 7860

# Run the application
CMD ["python", "app.py"]