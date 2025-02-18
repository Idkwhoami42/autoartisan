import { useEffect, useState } from "react";
import { Button } from "./components/ui/button";

export default function Resovoir() {
  const [volume, setVolume] = useState(100);
  const [color, setColor] = useState("green");
  const [refilled, setRefilled] = useState(false);

  useEffect(() => {
    const eventSource = new EventSource("http://localhost:5000/resovoir");

    eventSource.onmessage = (event) => {
      setVolume(parseInt(event.data));
    };

    eventSource.onerror = (error) => {
      console.error("EventSource failed:", error);
      eventSource.close(); // Close the connection on error
    };

    return () => {
      eventSource.close();
    };
  }, [refilled]);

  useEffect(() => {
    if (volume <= 10) setColor("red");
    else if (volume <= 25) setColor("orange");
    else if (volume <= 50) setColor("yellow");
    else setColor("green");
  }, [volume]);

  return (
    <>
      <div className="border-green-600"></div>
      <div className="border-orange-600"></div>
      <div className="border-red-600"></div>
      <div className="border-yellow-600"></div>
      <span className="text-2xl">Extruder</span>
      <span className="mt-4 text-xl">
        {volume != 0 ? (
          "Status: OK"
        ) : (
          <Button variant="secondary" className="text-xl mx-auto cursor-pointer" onClick={() => setRefilled(!refilled)}>
            Refill!
          </Button>
        )}
      </span>
      <span className="text-xl">Remaining: {volume}%</span>
      <div
        className={`mx-auto w-[60%] h-full bg-transparent border-3 border-${color}-600 flex flex-col-reverse gap-3 items-center px-3 py-2 my-4`}
      >
        {[...Array(Math.floor(volume / 10))].map((_, i) => (
          <div
            key={i}
            className={`h-[25px] w-full bg-transparent border-3 border-${color}-600 `}
          ></div>
        ))}
      </div>
    </>
  );
}
