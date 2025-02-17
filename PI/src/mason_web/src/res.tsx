import { useEffect, useState } from "react";

export default function Resovoir() {
  const [volume, setVolume] = useState(100);
  const [color, setColor] = useState("green");

  useEffect(() => {
    new Promise((resolve) => setInterval(resolve, 50)).then(() => {
      if (volume != 0) 
        setVolume((volume - 1));
    });
  }, [volume]);

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
      <span className="mt-4 text-xl">State: {volume != 0 ? "OK" : "Empty"}</span>
      <span className="text-xl">Remaining: {volume}%</span>
      <div
        className={`mx-auto w-[60%] h-full bg-transparent border-3 border-${color}-600 flex flex-col-reverse gap-3 items-center px-3 py-2 my-4`}
      >
        {[...Array(Math.floor(volume/10))].map((_, i) => (
          <div
            key={i}
            className={`h-[30px] w-full bg-transparent border-3 border-${color}-600 `}
          ></div>
        ))}
      </div>
    </>
  );
}
