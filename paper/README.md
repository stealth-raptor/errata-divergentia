# Paper draft

Draft of the paper on FOPSO-GWO tuning of the arm's FOPID controllers, in Springer's
proceedings class (`svproc`).

| File | Contents |
|---|---|
| [`main.tex`](main.tex), [`main.pdf`](main.pdf) | the manuscript and its compiled PDF |
| [`references.bib`](references.bib) | the bibliography (`spmpsci` style) |
| `svproc.cls`, `spmpsci.bst` | Springer's class and BibTeX style, from the author package |
| [`figures/`](figures) | the three figures (PDF) and [`make_figures.py`](figures/make_figures.py), which draws them |
| [`paper_numbers.m`](paper_numbers.m) | the one number that no stored result file holds |

## Building

```bash
cd paper
pdflatex main && bibtex main && pdflatex main && pdflatex main
```

The draft needs the LaTeX packages amsmath, graphicx, booktabs, algorithm and
algpseudocode; Overleaf has all of them.

## Where the numbers come from

| Paper | Source |
|---|---|
| Table 1 (arm) | `robot_params.m` |
| Table 2 (settings) | `hybrid_fopso_gwo.m` defaults, `tune_fopid_hybrid.m` |
| Fitness function, Eq. (9) | `fopid_fitness.m` (`whole`), constants in any stored run, e.g. `results/optimizer_runs/fopso_gwo_whole_seed6.mat` |
| Fig. 1 (convergence) | `results/optimizer_runs/fopso_gwo_whole_seed1..8.mat`; initial fitness 35.4: `paper_numbers.m` |
| Table 3 (tuned parameters) | `results/table_gains/summary.md` Sect. 8 |
| Tables 4–5, Figs. 2–3 | `results/table_gains/summary.md` Sects. 1, 3 and 5 and `results/table_gains/simulation_results.mat`, after `main('table')` |

To redraw the figures from the stored results (Python 3 with numpy, scipy and matplotlib):

```bash
python3 paper/figures/make_figures.py      # from the repository root
```

## Before submission

* Authors, affiliations and acknowledgements are placeholders (`TODO` in `main.tex`).
* Check every reference against the publisher's record.
* For a double-blind venue, keep the repository link out of the review copy.
