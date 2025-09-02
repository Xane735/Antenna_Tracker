# 1. Import necessary libraries
#    - FastAPI: The main class to create your web application.
#    - BaseModel: From Pydantic, used for defining the structure of request bodies (data validation).
#    - Optional: From typing, used to make query parameters optional.
from fastapi import FastAPI
from pydantic import BaseModel
from typing import Optional

# 2. Create an instance of the FastAPI application
#    This 'app' object is the main point of interaction for creating all your API endpoints.
app = FastAPI(
    title="My First FastAPI App",
    description="This is a simple skeleton app showing the basic building blocks.",
    version="1.0.0",
)

# --- Defining a Data Model (for POST/PUT requests) ---
# Pydantic models define the "shape" of your data. FastAPI uses them to:
#  a) Validate incoming request data.
#  b) Automatically generate documentation.
#  c) Provide type hints and autocompletion in your editor.
class Item(BaseModel):
    name: str
    description: Optional[str] = None # This field is optional and defaults to None
    price: float
    is_offer: Optional[bool] = None

# --- Creating API Endpoints (Routes) ---

# 3. Define the root endpoint
#    - A "decorator" (@app.get) tells FastAPI that the function below it is
#      responsible for handling requests for a specific path and HTTP method.
#    - 'async def' creates an asynchronous function, which is a modern and
#      efficient way to handle requests, especially those involving I/O
#      (like database calls or external API requests).
#    - Path: "/" (the root URL)
#    - HTTP Method: GET
@app.get("/")
async def read_root():
    """
    This is the root endpoint. It's a common practice to have a simple
    "hello world" or status message here.
    """
    return {"message": "Welcome to the FastAPI Skeleton App!"}

# 4. Define an endpoint with a path parameter
#    - The value of '{item_id}' in the path will be passed as an argument
#      to the function.
#    - Type hints (e.g., 'item_id: int') are crucial. FastAPI uses them to
#      validate the data type. If a user provides "foo" instead of an integer,
#      FastAPI will automatically return a clear error response.
#    - Path: e.g., /items/5
#    - HTTP Method: GET
@app.get("/items/{item_id}")
async def read_item(item_id: int):
    """
    Fetches an item by its ID. Demonstrates path parameters and type validation.
    """
    return {"item_id": item_id}

# 5. Define an endpoint with query parameters
#    - Parameters not in the path are automatically interpreted as query parameters.
#    - They are great for filtering, sorting, or pagination.
#    - Providing a default value (e.g., 'limit: int = 10') makes them optional.
#    - Path: e.g., /search/?query=books&limit=25
#    - HTTP Method: GET
@app.get("/search/")
async def search_items(query: str, limit: int = 10):
    """
    Searches for items. Demonstrates required and optional query parameters.
    """
    return {"query": query, "limit": limit}


# 6. Define an endpoint to create data (using POST)
#    - The POST method is used to send data to the server to create a new resource.
#    - The 'item: Item' parameter tells FastAPI to expect a JSON body that
#      matches the structure of the 'Item' Pydantic model we defined earlier.
#    - FastAPI will automatically parse the incoming JSON, validate it, and
#      convert it into an 'Item' object.
#    - Path: /items/
#    - HTTP Method: POST
@app.post("/items/")
async def create_item(item: Item):
    """
    Creates a new item. Demonstrates using a Pydantic model for the request body.
    """
    # In a real application, you would save this item to a database.
    # Here, we just return the data we received to confirm it worked.
    print(f"Received new item: {item.name}, Price: {item.price}")
    return {"status": "success", "item_created": item}

# How to run this application:
# 1. Save this file as 'main.py'.
# 2. Make sure you have fastapi and an ASGI server like uvicorn installed:
#    pip install "fastapi[all]"
# 3. In your terminal, run the following command from the same directory:
#    uvicorn main:app --reload
#
#    - 'main': the name of your Python file (main.py).
#    - 'app': the name of the FastAPI instance you created ('app = FastAPI()').
#    - '--reload': makes the server restart automatically after you change the code.
#
# 4. Open your browser and go to http://127.0.0.1:8000
# 5. Go to http://127.0.0.1:8000/docs to see the automatic interactive documentation!

import uvicorn
uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")