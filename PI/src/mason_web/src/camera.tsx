// keep polling localhost:5000/latest_image, returns jpeg base64 encoded image

import { useEffect, useState } from "react";

const Camera = () => {
  const [image, setImage] = useState<string>("");
  const [image2, setImage2] = useState<string>("");

  useEffect(() => {
    const eventSource = new EventSource("http://localhost:5000/latest_image");
    eventSource.onmessage = (event) => {
      const data = JSON.parse(event.data);
      setImage(data.image);
      setImage2(data.detection);
    };

    return () => {
      eventSource.close();
    };
  }, []);

  return (
    <>
      <span className="text-2xl">Camera </span>
      <div className="mt-3 flex gap-1">
        <div className="border-2 flex-1 h-[200px]">
          {image === "" ? (
            <div className="w-full h-full p-4">Waiting...</div>
          ) : (
            <img src={image} className="w-full h-full object-fill" />
          )}
        </div>
        <div className="border-2 flex-1 h-full">
          {image2 === "" ? (
            <div className="w-full h-full p-4">Waiting...</div>
          ) : (
            <img src={image2} className="w-full h-full object-fill" />
          )}
        </div>
      </div>
    </>
  );
};

export default Camera;
