# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information
import os
import subprocess
import sys
sys.path.insert(0, os.path.abspath('../src/tiago1/scripts'))
#subprocess.call('doxygen Doxyfile.in', shell=True)
show_authors = True

project = 'cogar_ass1'
copyright = '2025, Arian Tavousi'
author = 'Arian Tavousi'
release = '1.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
'sphinx.ext.autodoc',
'sphinx.ext.doctest',
'sphinx.ext.intersphinx',
"sphinx.ext.autosummary",
'sphinx.ext.todo',
'sphinx.ext.coverage',
'sphinx.ext.mathjax',
'sphinx.ext.ifconfig',
'sphinx.ext.viewcode',
'sphinx.ext.githubpages',
"sphinx.ext.napoleon",
'sphinx.ext.inheritance_diagram',
'sphinx_ros',
'sphinxcontrib.mermaid'
]


templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

autodoc_mock_imports = [
    "rospy",
    "std_srvs",
    "geometry_msgs",
    "nav_msgs",
    "actionlib",
    "tf",
    "sensor_msgs",
    "std_msgs",
    "actionlib_msgs"
    'sensor_msgs.msg',
    'nav_msgs.msg',
    'cv_bridge',
    'cv2',
    'numpy',
    'yaml',
    'tiago1.msg',
    'tiago1.srv',
    'cogar_ws',
    'sklearn',
    'std_msgs.msg',
    'tiago1',
    'os'
]




# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

highlight_language = 'python'
source_suffix = '.rst'
master_doc = 'index'
html_theme = 'sphinx_rtd_theme'
html_theme_options = {"navigation_depth": 3}
#html_static_path = ['_static']

autosummary_generate = True 

intersphinx_mapping = {'python': ('https://docs.python.org/3', None)}
todo_include_todos = True

napoleon_google_docstring = True
napoleon_numpy_docstring  = True 
autoclass_content = 'both'
toc_object_entries = False

mermaid_output_format = "png"
mermaid_params = ["--width", "600", "--backgroundColor", "transparent"]
