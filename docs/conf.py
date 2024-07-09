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
    (master_doc, 'cartographer-ros-doc-zh-cn', 'Cartographer ROS Documentation (Chinese)',
     [author], 1)
]

# -- Options for Texinfo output ----------------------------------------------

texinfo_documents = [
    (master_doc, 'cartographer-ros-doc-zh-cn', 'Cartographer ROS Documentation (Chinese)',
     author, 'cartographer-ros-doc-zh-cn', 'Documentation for Cartographer ROS (Chinese)',
     'Miscellaneous'),
]

# -- Extension configuration -------------------------------------------------

# Add custom CSS files to the static path and then define them in html_static_path
def setup(app):
    app.add_css_file('custom.css')

