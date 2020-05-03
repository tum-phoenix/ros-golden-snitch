from distutils.core import setup

setup(name='image_viewer',
    version='1.0',
    description='Viewer for ros image topics',
    url='https://github.com/orgs/tum-phoenix/teams/autonomous-flight',
    license='MIT',
    scripts=['view_topic'],
    packages=['image_viewer']
    )

