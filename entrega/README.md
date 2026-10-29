# Fuentes de la documentación

Los documentos finales están en la carpeta `Docs/` de la raíz:

- [Memoria](../Docs/memoria_final.pdf).
- [Presentación](../Docs/Presentacion.pdf).
- [Documentación del código](../Docs/documentacion_codigo.pdf).

Aquí se conservan sus fuentes editables: `paper/` para la memoria,
`presentacion/` para las diapositivas y `docs/` para la referencia Sphinx.

## Documentación del código

Ejecuta desde la raíz del repositorio:

```bash
python3 -m venv .venv-docs
.venv-docs/bin/python -m pip install --only-binary=:all: -r entrega/requirements-docs.txt
.venv-docs/bin/sphinx-build -W -b html entrega/docs entrega/docs/_build/html
```

Para producir el PDF se necesita LaTeX con `latexmk`:

```bash
.venv-docs/bin/sphinx-build -W -b latex entrega/docs entrega/docs/_build/latex
latexmk -cd -pdf -interaction=nonstopmode -halt-on-error entrega/docs/_build/latex/documentacion_codigo.tex
cp entrega/docs/_build/latex/documentacion_codigo.pdf Docs/documentacion_codigo.pdf
```

## Memoria y presentación

También desde la raíz, con LaTeX y `latexmk` instalados:

```bash
latexmk -cd -pdf -interaction=nonstopmode -halt-on-error entrega/paper/memoria_final.tex
latexmk -cd -pdf -interaction=nonstopmode -halt-on-error entrega/presentacion/presentacion_10min.tex
cp entrega/paper/memoria_final.pdf Docs/memoria_final.pdf
cp entrega/presentacion/presentacion_10min.pdf Docs/Presentacion.pdf
```

Los auxiliares de compilación se excluyen de Git. Las copias de `Docs/` se
actualizan con los comandos `cp` después de comprobar el resultado.
