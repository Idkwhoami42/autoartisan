import { useAtom } from "jotai";
import { Button } from "./components/ui/button";
import { masonStateAtom } from "./state";
import { stateToColor } from "./lib/utils";
import Resovoir from "./res";

const Dashboard = () => {
  const [masonState, setMasonState] = useAtom(masonStateAtom);

  return (
    <main className="min-h-screen flex flex-col font-mono">
      {masonState === "WAITING" && (
        <Button
          variant="ghost"
          className="text-4xl my-auto"
          onClick={() => {
            setMasonState("IDLE");
          }}
        >
          START MASON
        </Button>
      )}
      {masonState !== "WAITING" && <F />}
    </main>
  );
};

const F = () => {
  const [masonState, setMasonState] = useAtom(masonStateAtom);

  return (
    <div className="h-screen max-h-screen">
      <div
        className={`w-full flex justify-center items-center bg-${stateToColor(masonState)} text-3xl p-3`}
      >
        STATUS: {masonState !== "ERROR" ? "OPERATIONAL" : "ERROR"}
      </div>

      <div className="grid grid-cols-12 p-10 gap-4 grid-rows-15 max-h-[calc(100%-8rem)]">
        <div className="border-2 rounded-md p-3 text-2xl col-span-4 row-span-2">
          STATE: {masonState}
        </div>

        <div className="border-2 rounded-md p-4 text-2xl col-span-3 row-span-4 flex flex-col">
          <span className="text-2xl">Gantry</span>
          <span className="mt-4 text-xl">Length: 120 cm</span>
          <span className="text-xl">Width: 100 cm</span>
        </div>

        <div className="border-2 rounded-md p-4 text-2xl col-span-2 row-span-15 flex flex-col">
          <Resovoir />
        </div>

        <div className="border-2 rounded-md p-4 text-2xl col-span-3 row-span-15 flex flex-col">
          <span className="text-2xl">Settings </span>          
        </div>

        <div className="border-2 rounded-md p-4 text-2xl col-span-4 row-span-5 flex flex-col">
          <span className="text-2xl">Tool Heads</span>
          <div className="mt-4 flex gap-1 flex-col">
            <div className="flex justify-between">
              <div className="text-xl">Vertical Smoothing:</div>
              <div className="text-xl">Activated</div>
            </div>
            <div className="flex justify-between">
              <div className="text-xl">Horizontal Smoothing:</div>
              <div className="text-xl">Deactivated</div>
            </div>
            <div className="flex justify-between">
              <div className="text-xl">Brush:</div>
              <div className="text-xl">Deactivated</div>
            </div>
          </div>
        </div>

        <div className="border-2 rounded-md p-4 text-2xl col-span-3 row-span-3 flex flex-col">
          <span className="text-2xl">Position</span>
          <span className="mt-2 text-xl">X: {0}   Y: {0}</span>
        </div>

        <div className="border-2 rounded-md p-4 text-2xl col-span-7 row-span-9 flex flex-col">
          <span className="text-2xl">Camera </span>
          <div className="mt-3 flex gap-1">
            <div className="border-2 flex-1 h-[200px]"></div>
            <div className="border-2 flex-1 h-full"></div>
          </div>
        </div>
        
      </div>

      <div
        className={`w-full flex justify-center items-center mt-auto bg-${stateToColor(masonState)} text-3xl p-3`}
      >
        <Button
          variant="ghost"
          className="text-3xl hover:bg-transparent cursor-pointer"
          onClick={() => {
            if (masonState === "IDLE") {
              setMasonState("RUNNING");
            } else if (masonState === "RUNNING") {
              setMasonState("ERROR");
            }
          }}
        >
          {masonState === "IDLE" && "START MASON"}
          {masonState === "RUNNING" && "STOP MASON"}
          {masonState === "ERROR" && "ERROR, RESTART MASON"}
        </Button>
      </div>
    </div>
  );
};

export default Dashboard;
