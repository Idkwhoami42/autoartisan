import { useEffect, useState } from "react";

const Position = () => {
  const [position, setPosition] = useState<{ x: number; y: number }>({
    x: 0,
    y: 0,
  });

  useEffect(() => {
    const eventSource = new EventSource("http://localhost:5000/position");
    eventSource.onmessage = (event) => {
      const data = JSON.parse(event.data);
      setPosition({
        x: data.x,
        y: data.y,
      });
      console.log(data);
    };
  }, []);

  return (
    <>
      <span className="text-2xl">Position</span>
      <span className="mt-2 text-xl">
        X: {position.x.toFixed(2)} Y: {position.y.toFixed(2)}
      </span>
    </>
  );
};

export default Position;
