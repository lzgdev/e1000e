/**
 * e1000_watchdog - Timer Call-back
 * @data: pointer to adapter cast into an unsigned long
 **/
static void e1000_watchdog(unsigned long data)
{
	struct e1000_adapter *adapter = (struct e1000_adapter *)data;

	/* Do the rest outside of interrupt context */
	schedule_work(&adapter->watchdog_task);

	/* TODO: make this use queue_delayed_work() */
}

static void e1000_watchdog_task(struct work_struct *work)
{
	struct e1000_adapter *adapter = container_of(work,
						     struct e1000_adapter,
						     watchdog_task);
	struct net_device *netdev = adapter->netdev;
	struct e1000_mac_info *mac = &adapter->hw.mac;
	struct e1000_phy_info *phy = &adapter->hw.phy;
	struct e1000_ring *tx_ring = adapter->tx_ring;
	struct e1000_hw *hw = &adapter->hw;
	u32 link, tctl;

	if (test_bit(__E1000_DOWN, &adapter->state))
		return;

	link = e1000e_has_link(adapter);
	if ((netif_carrier_ok(netdev)) && link) {
		/* Cancel scheduled suspend requests. */
		pm_runtime_resume(netdev->dev.parent);

		e1000e_enable_receives(adapter);
		goto link_up;
	}

	if ((e1000e_enable_tx_pkt_filtering(hw)) &&
	    (adapter->mng_vlan_id != adapter->hw.mng_cookie.vlan_id))
		e1000_update_mng_vlan(adapter);

	if (link) {
		if (!netif_carrier_ok(netdev)) {
			bool txb2b = true;

			/* Cancel scheduled suspend requests. */
			pm_runtime_resume(netdev->dev.parent);

			/* update snapshot of PHY registers on LSC */
			e1000_phy_read_status(adapter);
			mac->ops.get_link_up_info(&adapter->hw,
						  &adapter->link_speed,
						  &adapter->link_duplex);
			e1000_print_link_info(adapter);

			/* check if SmartSpeed worked */
			e1000e_check_downshift(hw);
			if (phy->speed_downgraded)
				netdev_warn(netdev,
					    "Link Speed was downgraded by SmartSpeed\n");

			/* On supported PHYs, check for duplex mismatch only
			 * if link has autonegotiated at 10/100 half
			 */
			if ((hw->phy.type == e1000_phy_igp_3 ||
			     hw->phy.type == e1000_phy_bm) &&
			    hw->mac.autoneg &&
			    (adapter->link_speed == SPEED_10 ||
			     adapter->link_speed == SPEED_100) &&
			    (adapter->link_duplex == HALF_DUPLEX)) {
				u16 autoneg_exp;

				e1e_rphy(hw, MII_EXPANSION, &autoneg_exp);

				if (!(autoneg_exp & EXPANSION_NWAY))
					e_info("Autonegotiated half duplex but link partner cannot autoneg.  Try forcing full duplex if link gets many collisions.\n");
			}

			/* adjust timeout factor according to speed/duplex */
			adapter->tx_timeout_factor = 1;
			switch (adapter->link_speed) {
			case SPEED_10:
				txb2b = false;
				adapter->tx_timeout_factor = 16;
				break;
			case SPEED_100:
				txb2b = false;
				adapter->tx_timeout_factor = 10;
				break;
			}

			/* workaround: re-program speed mode bit after
			 * link-up event
			 */
			if ((adapter->flags & FLAG_TARC_SPEED_MODE_BIT) &&
			    !txb2b) {
				u32 tarc0;
				tarc0 = er32(TARC(0));
				tarc0 &= ~SPEED_MODE_BIT;
				ew32(TARC(0), tarc0);
			}

			/* disable TSO for pcie and 10/100 speeds, to avoid
			 * some hardware issues
			 */
			if (!(adapter->flags & FLAG_TSO_FORCE)) {
				switch (adapter->link_speed) {
				case SPEED_10:
				case SPEED_100:
					e_info("10/100 speed: disabling TSO\n");
					netdev->features &= ~NETIF_F_TSO;
					netdev->features &= ~NETIF_F_TSO6;
					break;
				case SPEED_1000:
					netdev->features |= NETIF_F_TSO;
					netdev->features |= NETIF_F_TSO6;
					break;
				default:
					/* oops */
					break;
				}
			}

			/* enable transmits in the hardware, need to do this
			 * after setting TARC(0)
			 */
			tctl = er32(TCTL);
			tctl |= E1000_TCTL_EN;
			ew32(TCTL, tctl);

			/* Perform any post-link-up configuration before
			 * reporting link up.
			 */
			if (phy->ops.cfg_on_link_up)
				phy->ops.cfg_on_link_up(hw);

			netif_carrier_on(netdev);

			if (!test_bit(__E1000_DOWN, &adapter->state))
				mod_timer(&adapter->phy_info_timer,
					  round_jiffies(jiffies + 2 * HZ));
		}
	} else {
		if (netif_carrier_ok(netdev)) {
			adapter->link_speed = 0;
			adapter->link_duplex = 0;
			/* Link status message must follow this format */
			pr_info("%s NIC Link is Down\n", adapter->netdev->name);
			netif_carrier_off(netdev);
			if (!test_bit(__E1000_DOWN, &adapter->state))
				mod_timer(&adapter->phy_info_timer,
					  round_jiffies(jiffies + 2 * HZ));

			/* The link is lost so the controller stops DMA.
			 * If there is queued Tx work that cannot be done
			 * or if on an 8000ES2LAN which requires a Rx packet
			 * buffer work-around on link down event, reset the
			 * controller to flush the Tx/Rx packet buffers.
			 * (Do the reset outside of interrupt context).
			 */
			if ((adapter->flags & FLAG_RX_NEEDS_RESTART) ||
			    (e1000_desc_unused(tx_ring) + 1 < tx_ring->count))
				adapter->flags |= FLAG_RESTART_NOW;
			else
				pm_schedule_suspend(netdev->dev.parent,
						    LINK_TIMEOUT);
		}
	}

link_up:
	spin_lock(&adapter->stats64_lock);
	e1000e_update_stats(adapter);

	mac->tx_packet_delta = adapter->stats.tpt - adapter->tpt_old;
	adapter->tpt_old = adapter->stats.tpt;
	mac->collision_delta = adapter->stats.colc - adapter->colc_old;
	adapter->colc_old = adapter->stats.colc;

	adapter->gorc = adapter->stats.gorc - adapter->gorc_old;
	adapter->gorc_old = adapter->stats.gorc;
	adapter->gotc = adapter->stats.gotc - adapter->gotc_old;
	adapter->gotc_old = adapter->stats.gotc;
	spin_unlock(&adapter->stats64_lock);

	if (adapter->flags & FLAG_RESTART_NOW) {
		schedule_work(&adapter->reset_task);
		/* return immediately since reset is imminent */
		return;
	}

	e1000e_update_adaptive(&adapter->hw);

	/* Simple mode for Interrupt Throttle Rate (ITR) */
	if (adapter->itr_setting == 4) {
		/* Symmetric Tx/Rx gets a reduced ITR=2000;
		 * Total asymmetrical Tx or Rx gets ITR=8000;
		 * everyone else is between 2000-8000.
		 */
		u32 goc = (adapter->gotc + adapter->gorc) / 10000;
		u32 dif = (adapter->gotc > adapter->gorc ?
			   adapter->gotc - adapter->gorc :
			   adapter->gorc - adapter->gotc) / 10000;
		u32 itr = goc > 0 ? (dif * 6000 / goc + 2000) : 8000;

		e1000e_write_itr(adapter, itr);
	}

	/* Cause software interrupt to ensure Rx ring is cleaned */
	if (adapter->msix_entries)
		ew32(ICS, adapter->rx_ring->ims_val);
	else
		ew32(ICS, E1000_ICS_RXDMT0);

	/* flush pending descriptors to memory before detecting Tx hang */
	e1000e_flush_descriptors(adapter);

	/* Force detection of hung controller every watchdog period */
	adapter->detect_tx_hung = true;

	/* Clear valid timestamp stuck in RXSTMPL/H due to a Rx error */
	if (adapter->hwtstamp_config.rx_filter != HWTSTAMP_FILTER_NONE) {
		if ((adapter->flags2 & FLAG2_CHECK_RX_HWTSTAMP) &&
		    (er32(TSYNCRXCTL) & E1000_TSYNCRXCTL_VALID)) {
			er32(RXSTMPH);
			adapter->rx_hwtstamp_cleared++;
		} else {
			adapter->flags2 |= FLAG2_CHECK_RX_HWTSTAMP;
		}
	}

	/* Reset the timer */
	if (!test_bit(__E1000_DOWN, &adapter->state))
		mod_timer(&adapter->watchdog_timer,
			  round_jiffies(jiffies + 2 * HZ));
}
