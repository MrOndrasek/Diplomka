#!/usr/bin/env python3
"""
Bezpečně omezené interaktivní ovládání KUKA LBR iiwa 14 R820 přes iiwaPy3/KST.

Tento program používá:
- aplikaci MatlabToolboxServer běžící na Sunrise controlleru,
- Python knihovnu iiwaPy3,
- TCP spojení na IP controlleru (výchozí 172.31.1.147).

Program po připojení neprovede žádný automatický pohyb.
Pohybové příkazy jsou dostupné až po explicitním příkazu `arm`
a každý pohyb vyžaduje potvrzení slovem MOVE.

Příklady:
    status
    arm
    jogj 7 1.0
    jogxyz 0 0 5
    disarm
    quit
"""

from __future__ import annotations

import argparse
import importlib
import math
import os
import sys
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence, Type


DEFAULT_IP = "172.31.1.147"
DEFAULT_PORT = 30001

# Tvrdé horní limity této aplikace. Argumenty programu je nemohou překročit.
HARD_MAX_JOINT_STEP_DEG = 5.0
HARD_MAX_CART_STEP_MM = 20.0
HARD_MAX_JOINT_VELOCITY_REL = 0.10
HARD_MAX_LINEAR_VELOCITY_MM_S = 50.0


@dataclass(frozen=True)
class Limits:
    joint_step_deg: float = 20.0
    cart_step_mm: float = 20.0
    joint_velocity_rel: float = 0.03
    linear_velocity_mm_s: float = 10.0


def load_iiwa_class(library_root: str | None) -> Type[Any]:
    """
    Načte třídu iiwaPy3 z několika běžných umístění repozitáře.

    Doporučené:
        git clone https://github.com/Modi1987/iiwaPy3.git ~/iiwaPy3
        export IIWAPY3_ROOT="$HOME/iiwaPy3"
    """
    candidates: list[Path] = []

    if library_root:
        candidates.append(Path(library_root).expanduser().resolve())

    env_root = os.environ.get("IIWAPY3_ROOT")
    if env_root:
        candidates.append(Path(env_root).expanduser().resolve())

    candidates.extend(
        [
            Path.cwd(),
            Path.home() / "iiwaPy3",
            Path.home() / "iiwaPy3-main",
            Path(__file__).resolve().parent / "iiwaPy3",
            Path(__file__).resolve().parent / "iiwaPy3-main",
        ]
    )

    errors: list[str] = []

    # Novější/robustnější import: kořen repozitáře je v sys.path
    # a python_client se načte jako namespace package.
    for root in candidates:
        if not root.exists():
            continue

        root_str = str(root)
        if root_str not in sys.path:
            sys.path.insert(0, root_str)

        try:
            module = importlib.import_module("python_client.iiwaPy3")
            return getattr(module, "iiwaPy3")
        except Exception as exc:
            errors.append(f"{root}: {exc}")

        # Fallback pro instalace, kde je python_client přímo v PYTHONPATH.
        client_dir = root / "python_client"
        if client_dir.exists():
            client_str = str(client_dir)
            if client_str not in sys.path:
                sys.path.insert(0, client_str)
            try:
                module = importlib.import_module("iiwaPy3")
                return getattr(module, "iiwaPy3")
            except Exception as exc:
                errors.append(f"{client_dir}: {exc}")

    # Poslední pokus pro již správně nastavené prostředí.
    for module_name in ("python_client.iiwaPy3", "iiwaPy3"):
        try:
            module = importlib.import_module(module_name)
            return getattr(module, "iiwaPy3")
        except Exception as exc:
            errors.append(f"{module_name}: {exc}")

    details = "\n".join(f"  - {line}" for line in errors[-8:])
    raise ImportError(
        "Nelze načíst knihovnu iiwaPy3.\n"
        "Naklonuj repozitář a nastav IIWAPY3_ROOT, například:\n"
        "  git clone https://github.com/Modi1987/iiwaPy3.git ~/iiwaPy3\n"
        "  export IIWAPY3_ROOT=\"$HOME/iiwaPy3\"\n"
        f"Poslední chyby importu:\n{details}"
    )


def require_vector(values: Sequence[Any], length: int, name: str) -> list[float]:
    result = [float(value) for value in values]
    if len(result) != length:
        raise RuntimeError(
            f"{name}: očekáváno {length} hodnot, přijato {len(result)}."
        )
    if not all(math.isfinite(value) for value in result):
        raise RuntimeError(f"{name}: robot vrátil neplatnou hodnotu.")
    return result


def format_joints(joints_rad: Sequence[float]) -> str:
    lines = ["Kloubové polohy:"]
    for index, value in enumerate(joints_rad, start=1):
        lines.append(
            f"  A{index}: {value:+.6f} rad  "
            f"({math.degrees(value):+.3f}°)"
        )
    return "\n".join(lines)


def format_eef(eef: Sequence[float]) -> str:
    return (
        "Poloha příruby/TCP:\n"
        f"  X={eef[0]:+.3f} mm, Y={eef[1]:+.3f} mm, Z={eef[2]:+.3f} mm\n"
        f"  α={eef[3]:+.6f} rad, β={eef[4]:+.6f} rad, γ={eef[5]:+.6f} rad"
    )


def confirm_motion(description: str) -> bool:
    print("\nPLÁNOVANÝ POHYB")
    print(description)
    print(
        "\nZkontroluj: režim T1, funkční E-STOP, volný pracovní prostor, "
        "správný nástroj/TCP a žádné aktivní bezpečnostní chyby."
    )
    answer = input("Pro provedení napiš přesně MOVE: ").strip()
    return answer == "MOVE"


class RobotSession:
    def __init__(self, robot_class: Type[Any], ip: str) -> None:
        self.robot_class = robot_class
        self.ip = ip
        self.robot: Any | None = None
        self.armed = False

    def __enter__(self) -> "RobotSession":
        print(f"Připojuji se k {self.ip}:{DEFAULT_PORT} ...")
        self.robot = self.robot_class(self.ip)

        # Konstruktor původní knihovny některé síťové chyby pouze vypíše,
        # proto spojení ověříme skutečným čtením stavu.
        joints = self.get_joints()
        eef = self.get_eef()
        print("Spojení navázáno.")
        print(format_joints(joints))
        print(format_eef(eef))
        return self

    def __exit__(self, exc_type, exc, tb) -> bool:
        self.armed = False
        if self.robot is not None:
            try:
                self.robot.setBlueOff()
            except Exception:
                pass
            try:
                self.robot.close()
                print("Spojení s MatlabToolboxServer bylo korektně ukončeno.")
            except Exception as close_error:
                print(
                    f"Varování: spojení se nepodařilo korektně ukončit: "
                    f"{close_error}",
                    file=sys.stderr,
                )
        return False

    def require_robot(self) -> Any:
        if self.robot is None:
            raise RuntimeError("Robot není připojen.")
        return self.robot

    def require_armed(self) -> None:
        if not self.armed:
            raise RuntimeError(
                "Pohyb je zamčený. Nejdřív použij příkaz `arm`."
            )

    def get_joints(self) -> list[float]:
        values = self.require_robot().getJointsPos()
        return require_vector(values, 7, "Kloubové polohy")

    def get_eef(self) -> list[float]:
        values = self.require_robot().getEEFPos()
        return require_vector(values, 6, "Poloha EEF")

    def print_status(self) -> None:
        print(format_joints(self.get_joints()))
        print(format_eef(self.get_eef()))
        print(f"Pohybové ovládání: {'ARMED' if self.armed else 'DISARMED'}")

    def arm(self) -> None:
        print(
            "\nPřed odemknutím musí být robot v T1, pracovní prostor volný, "
            "E-STOP funkční a bezpečnostní hlášky vyřešené."
        )
        token = input("Pro odemknutí napiš přesně ARM: ").strip()
        if token == "ARM":
            self.armed = True
            print("Pohybové ovládání je ARMED.")
        else:
            print("Odemknutí zrušeno.")

    def disarm(self) -> None:
        self.armed = False
        print("Pohybové ovládání je DISARMED.")

    def set_led(self, enabled: bool) -> None:
        robot = self.require_robot()
        if enabled:
            robot.setBlueOn()
        else:
            robot.setBlueOff()
        print(f"Modré světlo: {'zapnuto' if enabled else 'vypnuto'}.")

    def jog_joint(self, axis: int, delta_deg: float, limits: Limits) -> None:
        self.require_armed()

        if axis < 1 or axis > 7:
            raise ValueError("Číslo osy musí být 1 až 7.")
        if not math.isfinite(delta_deg):
            raise ValueError("Přírůstek musí být konečné číslo.")
        if abs(delta_deg) > limits.joint_step_deg:
            raise ValueError(
                f"Maximální krok je ±{limits.joint_step_deg:g}°."
            )

        current = self.get_joints()
        target = list(current)
        target[axis - 1] += math.radians(delta_deg)

        description = (
            f"Osa A{axis}: {math.degrees(current[axis - 1]):+.3f}° "
            f"→ {math.degrees(target[axis - 1]):+.3f}°\n"
            f"Relativní rychlost: {limits.joint_velocity_rel:.3f}"
        )
        if not confirm_motion(description):
            print("Pohyb zrušen.")
            return

        self.require_robot().movePTPJointSpace(
            target, [limits.joint_velocity_rel]
        )
        print("Pohyb dokončen.")
        print(format_joints(self.get_joints()))

    def jog_xyz(
        self, dx_mm: float, dy_mm: float, dz_mm: float, limits: Limits
    ) -> None:
        self.require_armed()

        delta = [float(dx_mm), float(dy_mm), float(dz_mm)]
        if not all(math.isfinite(value) for value in delta):
            raise ValueError("Kartézský přírůstek musí obsahovat konečná čísla.")

        distance = math.sqrt(sum(value * value for value in delta))
        if distance == 0:
            raise ValueError("Nulový pohyb nebude odeslán.")
        if distance > limits.cart_step_mm:
            raise ValueError(
                f"Délka kroku {distance:.3f} mm překračuje limit "
                f"{limits.cart_step_mm:g} mm."
            )

        description = (
            "Relativní pohyb TCP v BASE souřadnicích:\n"
            f"  ΔX={delta[0]:+.3f} mm, ΔY={delta[1]:+.3f} mm, "
            f"ΔZ={delta[2]:+.3f} mm\n"
            f"  délka={distance:.3f} mm, "
            f"rychlost={limits.linear_velocity_mm_s:.3f} mm/s"
        )
        if not confirm_motion(description):
            print("Pohyb zrušen.")
            return

        self.require_robot().movePTPLineEefRelBase(
            delta, [limits.linear_velocity_mm_s]
        )
        print("Pohyb dokončen.")
        print(format_eef(self.get_eef()))


HELP_TEXT = """
Příkazy:
  help
      Zobrazí tuto nápovědu.

  status
      Přečte kloubové polohy a polohu TCP/příruby.

  arm
      Odemkne pohybové příkazy po explicitním potvrzení.

  disarm
      Okamžitě zamkne další pohybové příkazy.

  jogj <osa 1-7> <přírůstek ve stupních>
      Malý relativní krok jednoho kloubu.
      Příklad: jogj 7 1.0

  jogxyz <dx_mm> <dy_mm> <dz_mm>
      Malý relativní lineární pohyb TCP v BASE souřadnicích.
      Příklad: jogxyz 0 0 5

  led on
  led off
      Ovládání modrého světla.

  quit
  exit
      Korektně pošle serveru příkaz end a ukončí spojení.
""".strip()


def interactive_loop(session: RobotSession, limits: Limits) -> None:
    print("\n" + HELP_TEXT)
    while True:
        try:
            line = input("\nkuka> ").strip()
        except EOFError:
            print()
            return

        if not line:
            continue

        parts = line.split()
        command = parts[0].lower()

        try:
            if command in {"quit", "exit"}:
                return
            if command == "help":
                print(HELP_TEXT)
            elif command == "status":
                session.print_status()
            elif command == "arm":
                session.arm()
            elif command == "disarm":
                session.disarm()
            elif command == "led":
                if len(parts) != 2 or parts[1].lower() not in {"on", "off"}:
                    raise ValueError("Použití: led on | led off")
                session.set_led(parts[1].lower() == "on")
            elif command == "jogj":
                if len(parts) != 3:
                    raise ValueError("Použití: jogj <osa 1-7> <delta_deg>")
                session.jog_joint(int(parts[1]), float(parts[2]), limits)
            elif command == "jogxyz":
                if len(parts) != 4:
                    raise ValueError(
                        "Použití: jogxyz <dx_mm> <dy_mm> <dz_mm>"
                    )
                session.jog_xyz(
                    float(parts[1]),
                    float(parts[2]),
                    float(parts[3]),
                    limits,
                )
            else:
                print("Neznámý příkaz. Použij `help`.")
        except (ValueError, RuntimeError) as exc:
            print(f"Chyba: {exc}")
        except KeyboardInterrupt:
            session.disarm()
            print("\nPříkaz přerušen; ovládání bylo zamknuto.")
        except Exception as exc:
            session.disarm()
            print(f"Neočekávaná chyba: {exc}", file=sys.stderr)
            traceback.print_exc()


def bounded(
    name: str, value: float, minimum: float, maximum: float
) -> float:
    if not math.isfinite(value) or not minimum <= value <= maximum:
        raise ValueError(
            f"{name} musí být v rozsahu {minimum:g} až {maximum:g}."
        )
    return value


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Interaktivní bezpečně omezené ovládání KUKA iiwa přes KST."
    )
    parser.add_argument("--ip", default=DEFAULT_IP, help="IP Sunrise controlleru")
    parser.add_argument(
        "--iiwapy-root",
        help="Kořen naklonovaného repozitáře iiwaPy3",
    )
    parser.add_argument(
        "--joint-step-deg",
        type=float,
        default=2.0,
        help=f"Maximální krok osy; nejvýše {HARD_MAX_JOINT_STEP_DEG}°",
    )
    parser.add_argument(
        "--cart-step-mm",
        type=float,
        default=5.0,
        help=f"Maximální kartézský krok; nejvýše {HARD_MAX_CART_STEP_MM} mm",
    )
    parser.add_argument(
        "--joint-velocity",
        type=float,
        default=0.03,
        help=(
            "Relativní kloubová rychlost; "
            f"nejvýše {HARD_MAX_JOINT_VELOCITY_REL}"
        ),
    )
    parser.add_argument(
        "--linear-velocity",
        type=float,
        default=10.0,
        help=(
            "Lineární rychlost v mm/s; "
            f"nejvýše {HARD_MAX_LINEAR_VELOCITY_MM_S}"
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    try:
        limits = Limits(
            joint_step_deg=bounded(
                "joint-step-deg",
                args.joint_step_deg,
                0.01,
                HARD_MAX_JOINT_STEP_DEG,
            ),
            cart_step_mm=bounded(
                "cart-step-mm",
                args.cart_step_mm,
                0.1,
                HARD_MAX_CART_STEP_MM,
            ),
            joint_velocity_rel=bounded(
                "joint-velocity",
                args.joint_velocity,
                0.001,
                HARD_MAX_JOINT_VELOCITY_REL,
            ),
            linear_velocity_mm_s=bounded(
                "linear-velocity",
                args.linear_velocity,
                0.1,
                HARD_MAX_LINEAR_VELOCITY_MM_S,
            ),
        )
        robot_class = load_iiwa_class(args.iiwapy_root)

        with RobotSession(robot_class, args.ip) as session:
            interactive_loop(session, limits)

        return 0
    except KeyboardInterrupt:
        print("\nProgram přerušen.")
        return 130
    except Exception as exc:
        print(f"Program nelze spustit: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())