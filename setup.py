try:
    from setuptools import setup
except ImportError:
    from distutils.core import setup

config = {
    'description' : 'A small module to estimate pi.',
    'author' : 'GURPGORK',
    'url' : 'URL to get it at.',
    'download_url' : 'Where to download it.',
    'author_email' : 'My email.',
    'version' : 0.1,
    'install_requires' : ['nose'],
    'packages' : ['collision'],
    'scripts' : [],
    'name' : 'collision'
}

setup(**config)
