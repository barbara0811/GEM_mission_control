import re
from gem_mission_control_lib.taems.taems import TaemsTree
from copy import deepcopy


class TAEMSFactory(object):
    # Find pattern [*X*] in a given string.
    PATTERN = r'(\[[^\]]*)X([^[]*\])'

    def __init__(self, template):
        self.template = template

    def instantiate(self, taems_id):
        """
        Replace template placeholders in tree with actual IDs.

        Args:
            taems_id (int): Mission ID.

        Returns:
            template_copy (TaemsTree): tames tree with actual mission ID.
        """
        template_copy = self.template.copy()

        # Replace placeholders with mission ID in each node.
        self._set_id(template_copy, template_copy.root, taems_id)
        replacement = r'\g<1>%s\g<2>' % taems_id
        template_copy.root_label = re.sub(TAEMSFactory.PATTERN, replacement, template_copy.root_label)

        IRs = {}
        for ir in template_copy.IRs.keys():
            label = re.sub(TAEMSFactory.PATTERN, replacement, ir)
            IRs[label] = template_copy.IRs[ir]
            IRs[label].label = label
            IRs[label].From = re.sub(TAEMSFactory.PATTERN, replacement, IRs[label].From)
            IRs[label].To = re.sub(TAEMSFactory.PATTERN, replacement, IRs[label].To)
        template_copy.IRs = IRs

        return template_copy

    def _set_id(self, tree, node, taems_id):
        """
        Replace template placeholders with actual ID in labels of each node.

        Args:
            node (Task): Task node for which replacement is being made.
            taems_id (int): Mission ID.
        """
        # Replace X in [*X*] with [*taems_id*] in labels of root task.
        replacement = r'\g<1>%s\g<2>' % taems_id
        if node.supertasks is not None:
            for i in range(len(node.supertasks)):
                node.supertasks[i] = re.sub(TAEMSFactory.PATTERN, replacement, node.supertasks[i])

        if node.subtasks is not None:
            # Call recursively for each child.
            st = deepcopy(node.subtasks)
            for label in st:
                if label not in tree.tasks.keys():
                    continue
                self._set_id(tree, tree.tasks[label], taems_id)
                
            for i in range(len(node.subtasks)):
                node.subtasks[i] = re.sub(TAEMSFactory.PATTERN, replacement, node.subtasks[i])

        old_label = node.label
        node.label = re.sub(TAEMSFactory.PATTERN, replacement, node.label)
        del tree.tasks[old_label]
        tree.tasks[node.label] = node

    @staticmethod
    def init_from_template(filename):
        """
        Initialize taems tree from a template file.

        Args:
            filename (str): Path to the file with template.

        Returns:
            TAEMSFactroy object that can be converted to an actual taems tree.
        """
        template = TaemsTree.load_from_file(filename)
        return TAEMSFactory(template)
