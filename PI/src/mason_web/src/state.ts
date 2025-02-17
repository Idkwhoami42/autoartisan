import { atom } from "jotai";

export type MasonState = "WAITING" | "IDLE" | "RUNNING" | "ERROR";

export const masonStateAtom = atom<MasonState>("WAITING");