import { useEffect, useState } from "react"

export default function Resovoir({ setOk }: { setOk: (ok: boolean) => void }) {
    const [items, setItems] = useState(10);
    const [color, setColor] = useState("green");

    useEffect(() => {
        new Promise((resolve) => setInterval(resolve, 1000)).then(() => {
            if (items != 0) {
                setItems(items - 1);
            }
        });
    }, [items]);

    useEffect(() => {
        if (items == 5) setColor("yellow");
        else if (items == 2) setColor("orange");
        else if (items == 0) {
            setColor("red");
            setOk(false);
        }
    }, [items]);


    return (
        <>
            <div className="border-green-600"></div>
            <div className="border-orange-600"></div>
            <div className="border-red-600"></div>
            <div className="border-yellow-600"></div>
            <div className={`w-[100px] h-[400px] bg-transparent border-4 border-${color}-600 flex flex-col-reverse gap-2 items-center py-2`}>
                {[...Array(items)].map((_, i) => (
                    <div key={i} className={`w-[85px] h-[30px] bg-transparent border-4 border-${color}-600`}></div>
                ))}
            </div>
        </>
    )
}