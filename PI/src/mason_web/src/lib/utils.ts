import { clsx, type ClassValue } from "clsx"
import { twMerge } from "tailwind-merge"
import { MasonState } from "~/state"

export function cn(...inputs: ClassValue[]) {
  return twMerge(clsx(inputs))
}


export function stateToColor(state: MasonState) {
  switch (state) {
    case "WAITING":
      return "gray-400"
    case "IDLE":
      return "green-400"
    case "RUNNING":
      return "green-400"
    case "ERROR":
      return "red-600"
  }
}