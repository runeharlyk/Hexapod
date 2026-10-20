"""The person at the robot during a hardware session (bench_floor.py, servo_envelope.py): asked in the terminal on the
robot, answered by a script in the simulation and the tests."""
from collections.abc import Callable


class Terminal:
    """Asks in the terminal. Every question but a confirmation can be skipped with Enter, which answers None."""

    def confirm(self, prompt: str) -> None:
        input(f"{prompt} [Enter] ")

    def number(self, prompt: str) -> float | None:
        while True:
            answer = input(f"{prompt} (Enter to skip): ").strip()
            if not answer:
                return None
            try:
                return float(answer)
            except ValueError:
                print(f"not a number: {answer!r}")

    def choice(self, prompt: str, options: dict[str, str]) -> str:
        """One of `options` by its key; Enter picks the first."""
        keys = list(options)
        menu = ", ".join(f"{key or 'Enter'} {meaning}" for key, meaning in options.items())
        while True:
            answer = input(f"{prompt} [{menu}]: ").strip().lower()
            if answer in options:
                return answer
            if not answer:
                return keys[0]
            print(f"one of: {', '.join(k for k in keys if k)}")


class Scripted:
    """Answers without a person: confirmations pass, numbers come from `numbers` by the prompt's first words (None when
    absent), and choices from `choose(prompt)`, or the first option."""

    def __init__(self, numbers: dict[str, float] | None = None, choose: Callable[[str], str] | None = None):
        self._numbers, self._choose = numbers or {}, choose
        self.asked: list[str] = []

    def confirm(self, prompt: str) -> None:
        self.asked.append(prompt)

    def number(self, prompt: str) -> float | None:
        self.asked.append(prompt)
        return next((value for start, value in self._numbers.items() if prompt.startswith(start)), None)

    def choice(self, prompt: str, options: dict[str, str]) -> str:
        self.asked.append(prompt)
        return self._choose(prompt) if self._choose is not None else next(iter(options))
