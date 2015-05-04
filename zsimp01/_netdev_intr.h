
// _netdev_intr_msix.h
static int e1000_request_msix(struct e1000_adapter *adapter);

// _netdev_intr_msi.h
static irqreturn_t e1000_intr_msi(int __always_unused irq, void *data);

// _netdev_intr_legacy.h
static irqreturn_t e1000_intr(int __always_unused irq, void *data);

void e1000e_reset_interrupt_capability(struct e1000_adapter *adapter)
{
	if (adapter->msix_entries) {
		pci_disable_msix(adapter->pdev);
		kfree(adapter->msix_entries);
		adapter->msix_entries = NULL;
	} else if (adapter->flags & FLAG_MSI_ENABLED) {
		pci_disable_msi(adapter->pdev);
		adapter->flags &= ~FLAG_MSI_ENABLED;
	}
}

/**
 * e1000e_set_interrupt_capability - set MSI or MSI-X if supported
 *
 * Attempt to configure interrupts using the best available
 * capabilities of the hardware and kernel.
 **/
void e1000e_set_interrupt_capability(struct e1000_adapter *adapter)
{
	int err;
	int i;

	switch (adapter->int_mode) {
	case E1000E_INT_MODE_MSIX:
		if (adapter->flags & FLAG_HAS_MSIX) {
			adapter->num_vectors = 3; /* RxQ0, TxQ0 and other */
			adapter->msix_entries = kcalloc(adapter->num_vectors,
							sizeof(struct
							       msix_entry),
							GFP_KERNEL);
			if (adapter->msix_entries) {
				for (i = 0; i < adapter->num_vectors; i++)
					adapter->msix_entries[i].entry = i;

				err = pci_enable_msix(adapter->pdev,
						      adapter->msix_entries,
						      adapter->num_vectors);
				if (err == 0)
					return;
			}
			/* MSI-X failed, so fall through and try MSI */
			e_err("Failed to initialize MSI-X interrupts.  Falling back to MSI interrupts.\n");
			e1000e_reset_interrupt_capability(adapter);
		}
		adapter->int_mode = E1000E_INT_MODE_MSI;
		/* Fall through */
	case E1000E_INT_MODE_MSI:
		if (!pci_enable_msi(adapter->pdev)) {
			adapter->flags |= FLAG_MSI_ENABLED;
		} else {
			adapter->int_mode = E1000E_INT_MODE_LEGACY;
			e_err("Failed to initialize MSI interrupts.  Falling back to legacy interrupts.\n");
		}
		/* Fall through */
	case E1000E_INT_MODE_LEGACY:
		/* Don't do anything; this is the system default */
		break;
	}

	/* store the number of vectors being used */
	adapter->num_vectors = 1;
}

/**
 * e1000_request_irq - initialize interrupts
 *
 * Attempts to configure interrupts using the best available
 * capabilities of the hardware and kernel.
 **/
static int e1000_request_irq(struct e1000_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;
	int err;

	if (adapter->msix_entries) {
		err = e1000_request_msix(adapter);
		if (!err)
			return err;
		/* fall back to MSI */
		e1000e_reset_interrupt_capability(adapter);
		adapter->int_mode = E1000E_INT_MODE_MSI;
		e1000e_set_interrupt_capability(adapter);
	}
	if (adapter->flags & FLAG_MSI_ENABLED) {
		err = request_irq(adapter->pdev->irq, e1000_intr_msi, 0,
				  netdev->name, netdev);
		if (!err)
			return err;

		/* fall back to legacy interrupt */
		e1000e_reset_interrupt_capability(adapter);
		adapter->int_mode = E1000E_INT_MODE_LEGACY;
	}

	err = request_irq(adapter->pdev->irq, e1000_intr, IRQF_SHARED,
			  netdev->name, netdev);
	if (err)
		e_err("Unable to allocate interrupt, Error: %d\n", err);

	return err;
}

static void e1000_free_irq(struct e1000_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;

	if (adapter->msix_entries) {
		int vector = 0;

		free_irq(adapter->msix_entries[vector].vector, netdev);
		vector++;

		free_irq(adapter->msix_entries[vector].vector, netdev);
		vector++;

		/* Other Causes interrupt vector */
		free_irq(adapter->msix_entries[vector].vector, netdev);
		return;
	}

	free_irq(adapter->pdev->irq, netdev);
}

/**
 * e1000_irq_disable - Mask off interrupt generation on the NIC
 **/
static void e1000_irq_disable(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;

	ew32(IMC, ~0);
	if (adapter->msix_entries)
		ew32(EIAC_82574, 0);
	e1e_flush();

	if (adapter->msix_entries) {
		int i;
		for (i = 0; i < adapter->num_vectors; i++)
			synchronize_irq(adapter->msix_entries[i].vector);
	} else {
		synchronize_irq(adapter->pdev->irq);
	}
}

/**
 * e1000_irq_enable - Enable default interrupt generation settings
 **/
static void e1000_irq_enable(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;

	if (adapter->msix_entries) {
		ew32(EIAC_82574, adapter->eiac_mask & E1000_EIAC_MASK_82574);
		ew32(IMS, adapter->eiac_mask | E1000_IMS_OTHER | E1000_IMS_LSC);
	} else if (hw->mac.type == e1000_pch_lpt) {
		ew32(IMS, IMS_ENABLE_MASK | E1000_IMS_ECCER);
	} else {
		ew32(IMS, IMS_ENABLE_MASK);
	}
	e1e_flush();
}

#include "_netdev_intr_msix.h"
#include "_netdev_intr_msi.h"
#include "_netdev_intr_legacy.h"

