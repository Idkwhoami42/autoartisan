// keep polling localhost:5000/latest_image, returns jpeg base64 encoded image

import { useEffect, useState } from "react";

export default function CameraFeed() {
  const [image, setImage] = useState<string>("");
  const [detectionImage, setDetectionImage] = useState<string>("");

  useEffect(() => {
    fetch("http://localhost:5000/latest_image")
      .then((response) => response.json())
      .then((data) => {
        setImage(data[0]);
        setDetectionImage(data[1]);
      });
  }, [image]);

  return (
    <>
      <div className="w-full h-full border-2 border-white">
        {image === "" ? (
          <div className="w-full h-full bg-black text-white">Waiting...</div>
        ) : (
          <img src={image} className="w-full h-full object-fill" />
        )}
      </div>
      <div className="w-full h-full border-2 border-white">
        {detectionImage === "" ? (
          <div className="w-full h-full bg-black text-white">Waiting...</div>
        ) : (
          <img src={detectionImage} className="w-full h-full object-fill" />
        )}
      </div>
    </>
  );
}
