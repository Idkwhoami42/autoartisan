# Mason Web

## Description

This folder contains the code for the web interface for the mason robot. The code is written in TypeScript and uses React and Bun. Additionally, it uses Vite for bundling and tailwind-css for styling. This is a single page application and thus has no routing.

## Usage

To run the web interface, you must have bun installed and run the following commands

```bash
# installing node dependencies
bun i
# starting the client
bun run dev --host autoartisan.local
```

The interface will be available at http://autoartisan.local:5173

## Style Guide and Linting

We use prettier and ESLint to format the code.
