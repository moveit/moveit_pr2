import sys, os

sys.path += [ os.path.abspath( 'doc' )]

extensions = [ 'sphinx.ext.extlinks',
               'tutorialformatter' ]

# The master toctree document.
master_doc = 'index'

# The suffix of source filenames.
source_suffix = '.rst'

project = u'pr2_moveit_tutorials'

copyright = u'2013,  SRI International'

# If true, sectionauthor and moduleauthor directives will be shown in the
# output. They are ignored by default.
show_authors = True

# The name of the Pygments (syntax highlighting) style to use.
pygments_style = 'sphinx'

extlinks = {'codedir': ('https://github.com/ros-planning/moveit_pr2/blob/hydro-devel/pr2_moveit_tutorials/%s', ''),
            'moveit_core': ('http://docs.ros.org/api/moveit_core/html/classmoveit_1_1core_1_1%s.html', '')}
