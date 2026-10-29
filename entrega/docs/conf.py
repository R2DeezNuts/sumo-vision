"""Configuracion Sphinx para documentar Sumo Vision."""

from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[2]

for rel_path in (
    ".",
    "blocks/0_recibir_sumocam",
    "blocks/1_procesar_imagen",
    "blocks/2_toma_decision",
    "blocks/3_enviar_respuesta",
):
    sys.path.insert(0, str(PROJECT_ROOT / rel_path))

project = "Sistema de vision para sumobot"
author = "Juan Jose Prado Neira"
copyright = "2026, Juan Jose Prado Neira"
release = "1.0"

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
]

autodoc_mock_imports = ["cv2", "numpy"]
autodoc_default_options = {
    "members": True,
    "undoc-members": True,
    "show-inheritance": True,
}
napoleon_google_docstring = True
napoleon_numpy_docstring = True

templates_path = ["_templates"]
exclude_patterns = ["_build"]
language = "es"

latex_engine = "pdflatex"
latex_documents = [
    (
        "index",
        "documentacion_codigo.tex",
        "Documentacion del codigo del sistema de vision para sumobot",
        author,
        "howto",
    ),
]
latex_elements = {
    "papersize": "a4paper",
    "pointsize": "10pt",
    "printindex": "",
}
latex_domain_indices = False
latex_use_index = False
