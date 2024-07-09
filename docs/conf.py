# -*- coding: utf-8 -*-

import os
import sys
sys.path.insert(0, os.path.abspath('.'))

# -- Project information -----------------------------------------------------

project = 'cartographer-ros-doc-zh-cn'
copyright = 'Copyright (c) 2024'
author = 'tabbleman'
master_doc = 'index'

# -- General configuration ---------------------------------------------------

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
]

templates_path = ['_templates']
exclude_patterns = []

# -- Options for HTML output -------------------------------------------------

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

# -- Options for LaTeX output ------------------------------------------------

latex_elements = {
}

# -- Options for manual page output ------------------------------------------

man_pages = [
    (master_doc, 'yourprojectname', 'Your Project Documentation',
     [author], 1)
]

# -- Options for Texinfo output ----------------------------------------------

texinfo_documents = [
    (master_doc, 'YourProjectName', 'Your Project Documentation',
     author, 'YourProjectName', 'One line description of project.',
     'Miscellaneous'),
]

# -- Extension configuration -------------------------------------------------

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# -- Options for HTMLHelp output ---------------------------------------------

htmlhelp_basename = 'YourProjectNamedoc'

# -- Other configuration ----------------------------------------------------

# At the bottom of conf.py
def setup(app):
    app.add_stylesheet('custom.css')

