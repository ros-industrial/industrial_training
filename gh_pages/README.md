# ROS-Industrial Training Site

### Building and viewing locally:

- Install Sphinx and dependencies (use of a python virtual environment encouraged):
```bash
    pip3 install --user --upgrade pip
    pip3 install --user Sphinx python-git recommonmark
```

- Build the html:
``` bash
   sphinx-build gh_pages/ build/
```
- Start a local HTTP server:
```bash
    cd build/
    python3 -m http.server
```
- Browse to `localhost:8000` in your browser to inspect the site
