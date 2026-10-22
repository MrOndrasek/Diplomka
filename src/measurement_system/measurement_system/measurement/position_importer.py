import csv
import math

def load_positions(file_path):
    positions=[]
    point_ids=[]

    joint_names = [f"q{i}" for i in range(1,8)]

    with open(file_path,"r",encoding="utf-8-sig",newline="") as file:
        reader=csv.DictReader(file)

        if reader.fieldnames is None:
            raise ValueError("CSV soubor je prázdný")

        required= ["point_id"] + joint_names
        missing = [
            name for name in required
            if name not in reader.fieldnames
        ]

        if missing:
            raise ValueError(
                f'chybejici sloupce: {missing}'
            )

        for row_number, row in enumerate(reader, start=2):
            try:
                point_id=row["point_id"].strip()

                if not point_id:
                    raise ValueError("Prázdné point_id")

                if point_id in point_ids:
                    raise ValueError(
                        f'Duplicitní point_id:{point_id}'
                    )

                joints=[
                    float(row[name])
                    for name in joint_names
                ]

                if not all(math.isfinite(q) for q in joints):
                    raise ValueError("Neplatná hodnota úhlu")

                point_ids.append(point_id)
                positions.append(joints)

            except(ValueError,TypeError,KeyError) as e:
                raise ValueError(
                    f'Chyba na řádku {row_number}:{e}'
                ) from e

        if not positions:
            raise ValueError("CSV neobsahuje žádné pozice")
        return positions,point_ids