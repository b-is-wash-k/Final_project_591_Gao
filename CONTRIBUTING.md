# Team workflow

This is the workflow we're using so our work combines smoothly. Each change is made on its own branch and joins `main` through a Pull Request, so `main` always has a working version of the project.

## Getting the code

```bash
git clone https://github.com/b-is-wash-k/Final_project_591_Gao.git
```

## Working on a task

```bash
git switch main && git pull            # start from the latest main
git switch -c <name>/<task>            # e.g. andrea/llm-planner
# ... make changes, then:
git add <files>
git commit -m "Short description of the change"
git push -u origin <name>/<task>
```

Next, open a **Pull Request** on GitHub. Once another team member has looked it over, it can be merged into `main`.

## Bringing in the latest changes

```bash
git switch main && git pull
git switch <name>/<task> && git merge main
```

Git combines changes automatically in most cases. A conflict happens only when two people edit the same lines. To resolve one:
1. Open each file Git lists.
2. Between `<<<<<<<` and `>>>>>>>`, keep the version you want and remove the markers.
3. Run `git add <file>`, then `git commit`.

## Things that help

- Each part of the project has its own folder, so changes rarely overlap. Changes to another part can go through a Pull Request so its owner can see them.
- Shared formats (scenario file, topic names, world-state JSON) affect everyone, so changes to them can be discussed in a Pull Request first.
- Small Pull Requests are easier to review. Running `git pull` at the start of a work session avoids most conflicts.
- Build output, logs and large files are kept out of the repository (see `.gitignore`).
