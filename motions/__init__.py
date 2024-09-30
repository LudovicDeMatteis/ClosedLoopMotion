from .jump_battobot_closed import jump_battobot_closed
from .jump_battobot_open import jump_battobot_open
from .jump_digit_closed import jump_digit_closed

from .walk_battobot_closed import walk_battobot_closed
from .walk_battobot_open import walk_battobot_open
from .walk_digit_closed import walk_digit_closed

# from .sidewalk_battobot_closed import sidewalk_battobot_closed
# from .sidewalk_battobot_open import sidewalk_battobot_open
# from .sidewalk_digit_closed import sidewalk_digit_closed

# from .stairs_battobot_closed import stairs_battobot_closed
# from .stairs_battobot_open import stairs_battobot_open
# from .stairs_digit_closed import stairs_digit_closed

scripts = {
    "Jump": {
        "BattoBot Closed": jump_battobot_closed,
        "BattoBot Open": jump_battobot_open,
        "Digit Closed": jump_digit_closed,
    },
    "Walk": {
        "BattoBot Closed": walk_battobot_closed,
        "BattoBot Open": walk_battobot_open,
        "Digit Closed": walk_digit_closed,
    },
    # "Side walk": {
    #     "BattoBot Closed": sidewalk_battobot_closed,
    #     "BattoBot Open": sidewalk_battobot_open,
    #     "Digit Closed": sidewalk_digit_closed,
    # },
    # "Walk stairs": {
    #     "BattoBot Closed": stairs_battobot_closed,
    #     "BattoBot Open": stairs_battobot_open,
    #     "Digit Closed": stairs_digit_closed,
    # },
}