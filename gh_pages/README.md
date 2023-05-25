# ROS-Industrial Training Site

### Building and viewing locally:

- (Recommended) Create a python virtual environment for Sphinx packages and dependencies:

    ```
    cd <industrial_training root directory>
    python3 -m venv/ venv/
    source venv/bin/activate
    ```

- Install Sphinx and dependencies:
    ```
    pip3 install --upgrade pip
    pip3 install Sphinx==6.2.1 gitpython recommonmark
    ```

- Build the html:
    ```
    sphinx-build gh_pages/ build/
    ```

- Start a local HTTP server:
    ```
    cd build/
    python3 -m http.server
    ```

- Browse to `localhost:8000` in your browser to inspect the site.
