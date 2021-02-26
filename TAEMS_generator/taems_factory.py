import re
from taems_tree import TAEMSTree, TAEMSNode


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
            template_copy (TAEMSTree): tames tree with actual mission ID.
        """
        template_copy = self.template.copy()

        # Replace placeholders with mission ID in each node.
        self._set_id(template_copy.root, taems_id)

        # Replace X in [*X*] with [*taems_id*] for every task label in the tree.
        replacement = r'\g<1>%d\g<2>' % taems_id
        for i in range(len(template_copy.taskLabels)):
            template_copy.taskLabels[i] = re.sub(TAEMSFactory.PATTERN, replacement, template_copy.taskLabels[i])

        return template_copy

    def _set_id(self, node, taems_id):
        """
        Replace template placeholders with actual ID in labels of each node.

        Args:
            node (TAEMSNode): Task node for which replacement is being made.
            taems_id (int): Mission ID.
        """
        # Replace X in [*X*] with [*taems_id*] in labels of root task.
        replacement = r'\g<1>%d\g<2>' % taems_id
        node.label = re.sub(TAEMSFactory.PATTERN, replacement, node.label)
        # node.agent = re.sub(TAEMSFactory.PATTERN, replacement, node.agent)
        for i in range(len(node.childrenLabels)):
            node.childrenLabels[i] = re.sub(TAEMSFactory.PATTERN, replacement, node.childrenLabels[i])

        if node.super_tasks is not None:
            node.super_tasks = re.sub(TAEMSFactory.PATTERN, replacement, node.super_tasks)

        # Call recursively for each child.
        for child in node.children:
            self._set_id(child, taems_id)

    @staticmethod
    def init_from_template(filename):
        """
        Initialize taems tree from a template file.

        Args:
            filename (str): Path to the file with template.

        Returns:
            TAEMSFactroy object that can be converted to an actual taems tree.
        """
        template = TAEMSTree.load_from_file(filename)
        return TAEMSFactory(template)
