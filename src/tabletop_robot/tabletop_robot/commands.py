from dataclasses import dataclass
import re


@dataclass(frozen=True)
class SortCommand:
    color: str
    destination: str


COLORS = {'red': 'red', 'красный': 'red', 'красного': 'red',
          'green': 'green', 'зелёный': 'green', 'зеленый': 'green',
          'blue': 'blue', 'синий': 'blue'}
DESTINATIONS = {'left': 'left', 'левую': 'left', 'лево': 'left',
                'right': 'right', 'правую': 'right', 'право': 'right'}


def parse_command(text: str) -> SortCommand:
    words = re.findall(r'[а-яёa-z]+', text.lower())
    colors = {COLORS[word] for word in words if word in COLORS}
    destinations = {DESTINATIONS[word] for word in words if word in DESTINATIONS}
    if len(colors) != 1 or len(destinations) != 1:
        raise ValueError('Specify exactly one color and one destination: red left')
    return SortCommand(colors.pop(), destinations.pop())
