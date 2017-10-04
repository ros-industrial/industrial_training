# industrial_training
This branch is an orphan used to store GitHub pages for the repository website.

## Update documentation
 - Install tool to generate documentation for multiple versions from [here](https://robpol86.github.io/sphinxcontrib-versioning/master/install.html).
 - In each of the repositories make sure to update branch name in **gh_pages/conf.py** at each location below.
   - `scv_banner_main_ref = 'kinetic'`
   - `scv_whitelist_branches = ('kinetic','kinetic-devel')`
   - `version = u'kinetic'`
   - `release = u'kinetic'`
   - `github_version": "kinetic"`
 - Next push all changes to each of the branches to the repositories.
 - Build Documentation (Note: Change **kinetic** to the latest version)
   - Navigate to the repository.
   - `sphinx-versioning build -r kinetic gh_pages gh_pages/_build/html`
 - Copy the contents of **gh_pages/_build/html** to the **master** branch and push the changes.
