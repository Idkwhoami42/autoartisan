import { useState } from "react";
import CameraFeed from "./camerafeed";
import Resovoir from "./res";

function App() {


  const [ok, setOk] = useState(true);

  return (
    <main className="min-h-screen flex-col bg-black font-mono">
      <div className={`w-full flex justify-center items-center ${ok ? 'bg-[#32a887]' : 'bg-red-600'} text-4xl p-3`}>
        Status: Operational
      </div>
      <div className="flex text-white mx-10 my-5 w-full gap-15">
        <div className="h-full w-[20%] flex-col items-center gap-4">
          <div className="flex">
            <span>
              Extruded Amount
            </span>
            <span className="ml-auto">0.0L</span>
          </div>
          <div className="flex">
            <span>Amount Remaining</span>
            <span className="ml-auto">0.0L</span>
          </div>
          <div className="flex">
            <span>Runtime</span>
            <span className="ml-auto">00 h : 00 m : 00 s</span>
          </div>
          <div className="h-[40px]"></div>
          <span className="text-xl">Extruder Coordinates</span>
          <div className="flex gap-[20px]">
            <span className="text-red-600">X: 0.0</span>
            <span className="text-green-600">Y: 0.0</span>
          </div>

          <CameraFeed />

        </div>

        {/* <Resovoir setOk={setOk} /> */}

      </div>

    </main>
  )
}

export default App
